"""Bounded optimizer-input capture; no ROS or flight execution.

A pending record is persisted BEFORE IPOPT so worker termination still leaves
all optimizer inputs. The backup warm start is solver state, not a solve argument.
"""

import hashlib
import json
import logging
import mmap
import os
from pathlib import Path
import queue
import threading
import time

import numpy as np

from .upstream import provenance


def serializable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: serializable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [serializable(v) for v in value]
    return value


class CaptureJournal:
    """File-backed pending inputs and an independent completed-record writer.

    Ext4 rename/writeback can block for longer than a worker deadline. Prepare
    two mapped input files before readiness, then update resident pages before
    each solve. A killed worker leaves those ordinary, space-padded JSON files
    in the run directory. Completed records retain the atomic JSON ring format,
    with filesystem writes owned by a bounded background queue.
    """

    def __init__(self, directory, name, limit):
        self.root = Path(directory, name)
        self.root.mkdir(parents=True, exist_ok=True)
        self.sources = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in Path(__file__).parent.glob('*.py')}
        self.inputs = []
        self.lengths = [0, 0]
        self.capacity = 1024 * 1024
        for index in range(2):
            path = self.root / f'pending-{index}.json'
            with path.open('x+b') as stream:
                stream.truncate(self.capacity)
                mapped = mmap.mmap(stream.fileno(), self.capacity)
            mapped[:] = b' ' * self.capacity  # Prefault before worker readiness.
            initial = json.dumps(dict(schema=1, outcome='waiting', vehicle=name)).encode()
            mapped[:len(initial)] = initial
            self.lengths[index] = len(initial)
            self.inputs.append(mapped)
        self.queue = queue.Queue(maxsize=limit)
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._writer, name=f'captures-{name}', daemon=True)
        self.thread.start()

    def pending(self, sequence, record):
        payload = json.dumps(record, allow_nan=False).encode()
        if len(payload) > self.capacity:
            raise ValueError('Optimizer input exceeds the prepared capture journal capacity')
        index = sequence % 2
        mapped = self.inputs[index]
        mapped[:len(payload)] = payload
        if self.lengths[index] > len(payload):
            mapped[len(payload):self.lengths[index]] = b' ' * (self.lengths[index] - len(payload))
        self.lengths[index] = len(payload)

    def complete(self, path, record):
        payload = json.dumps(record, allow_nan=False)
        try:
            self.queue.put_nowait((path, payload))
        except queue.Full:
            logging.error('Optimizer capture queue full for %s; completed record omitted; '
                          'latest inputs remain in pending journals', path.parent.name)

    def _writer(self):
        while not self.stopping.is_set() or not self.queue.empty():
            try:
                path, payload = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                temporary = path.with_suffix('.tmp')
                temporary.write_text(payload)
                os.replace(temporary, path)
            except OSError:
                logging.exception('Optimizer capture write failed: %s; pending inputs retained', path)
            finally:
                self.queue.task_done()

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=1.0)
        if self.thread.is_alive():
            logging.error('Optimizer capture writer did not drain: %s; pending inputs retained', self.root)
        for mapped in self.inputs:
            mapped.close()


class SolverCapture:
    _sequences = {}
    _journals = {}

    @classmethod
    def prepare(cls, cfg, name):
        directory, limit = cfg.get('DIAGNOSTIC_DIR'), cfg.get('DIAGNOSTIC_CAPTURE_CYCLES', 128)
        if directory and limit > 0:
            key = (str(directory), name)
            if key not in cls._journals:
                cls._journals[key] = CaptureJournal(directory, name, limit)

    @classmethod
    def close(cls):
        for journal in cls._journals.values():
            journal.close()
        cls._journals.clear()

    def __init__(self, cfg, name, solver, stamp, inputs):
        self.path = None
        directory = cfg.get('DIAGNOSTIC_DIR')
        if not directory:
            return
        limit = cfg.get('DIAGNOSTIC_CAPTURE_CYCLES', 128)
        if limit <= 0:
            return
        key = (str(directory), name)
        sequence = self._sequences.get(key, 0) + 1
        self._sequences[key] = sequence
        self.prepare(cfg, name)
        self.journal = self._journals[key]
        root = self.journal.root
        self.path = root / f'solve-{sequence % limit:04d}.json'
        self.record = serializable(dict(
            schema=1, sequence=sequence, stamp=stamp, wall=time.time(), vehicle=name,
            cfg=cfg, n_others=solver.n_others, n_faces=solver.n_faces,
            inputs=inputs, backup_init=solver._bk_init, outcome='pending',
            upstream=provenance(),
            sources=self.journal.sources,
        ))
        self.journal.pending(sequence, self.record)

    def finish(self, ok, X, U, solver):
        if self.path is not None:
            self.record.update(outcome='success' if ok else 'failure', X=serializable(X),
                               U=serializable(U), backup=serializable(solver.last_backup))
            try:
                self.record['stats'] = serializable(solver.opti.stats())
            except RuntimeError:
                pass
            self.journal.complete(self.path, self.record)


def replay(path, iteration_only=False):
    """Replay a complete capture using its solver options; never starts ROS."""
    from .mpc_problem import MPCProblem
    record = json.loads(Path(path).read_text())
    if record.get('schema') != 1:
        raise ValueError('Unsupported optimizer capture schema')
    if record.get('outcome') == 'waiting':
        raise ValueError('This pending journal has not received a solver call yet')
    if iteration_only:
        record['cfg']['IPOPT_OPTIONS'].pop('ipopt.max_cpu_time', None)
    solver = MPCProblem(record['n_others'], record['n_faces'], record['cfg'])
    solver._bk_init = (None if record['backup_init'] is None else
                       tuple(np.asarray(v) for v in record['backup_init']))
    inputs = record['inputs']
    for key in ('X_init', 'U_init'):
        inputs[key] = np.asarray(inputs[key], float)
    return solver.solve(**inputs)
