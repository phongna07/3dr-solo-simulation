"""ROS-free persistent process transport, phase barriers and bounded shutdown.

Payloads belong to the caller. No planner, solver or numerical diagnostics load
here; the spawn target and optional timeout observer are supplied by composition.
"""

from dataclasses import dataclass
import multiprocessing as mp
from multiprocessing.connection import wait
import time


@dataclass
class Worker:
    process: object
    connection: object
    pid: int | None = None
    error: str | None = None


class WorkerPool:
    """Caller serializes dispatch and close; disabled workers never restart."""

    def __init__(self, names, target, args, timeout, startup_timeout,
                 on_timeout=None, process_prefix='uav'):
        self.timeout = timeout
        self.on_timeout = on_timeout
        self.workers = {}
        self.cycle = 0
        self.closed = False
        context = mp.get_context("spawn")
        try:
            for n in names:
                parent, child = context.Pipe()
                process = context.Process(target=target, args=(child, n, *args),
                                          name=f"{process_prefix}-{n}", daemon=True)
                process.start()
                child.close()
                self.workers[n] = Worker(process, parent)
            ready = self._collect(set(names), "ready", None, startup_timeout)
            for n, message in ready.items():
                self.workers[n].pid = message.get("pid")
                if "error" in message:
                    raise RuntimeError(f"{n}: worker initialization failed: {message['error']}")
        except BaseException:
            self.close()
            raise

    def _disable(self, name, reason):
        worker = self.workers[name]
        worker.error = reason
        if reason.startswith('worker_timeout:') and self.on_timeout is not None:
            self.on_timeout(worker)
        worker.connection.close()
        if worker.process.is_alive():
            worker.process.terminate()
        worker.process.join(timeout=0.05)
        if worker.process.is_alive():
            worker.process.kill()
            worker.process.join(timeout=0.1)

    def _collect(self, names, phase, cycle, timeout):
        """Collect whichever worker finishes first; one deadline for the phase."""
        pending = {self.workers[n].connection: n for n in names}
        messages = {}
        deadline = time.perf_counter() + timeout
        while pending:
            # A descheduled coordinator can wake after the deadline with timely
            # replies already queued. Drain those before disabling any worker;
            # worker completion timestamps still enforce the original deadline.
            remaining = max(0.0, deadline - time.perf_counter())
            readable = wait(list(pending), timeout=remaining)
            if not readable:
                break
            for connection in readable:
                name = pending.pop(connection)
                try:
                    message = connection.recv()
                    if message.get("phase") != phase or (
                        cycle is not None and message.get("cycle") != cycle
                    ):
                        raise ValueError("Worker response does not match the active cycle")
                    if message["finished_wall"] > deadline:
                        reason = f"worker_timeout: {phase}"
                        self._disable(name, reason)
                        messages[name] = {"error": reason}
                    else:
                        messages[name] = message
                except (EOFError, OSError, ValueError) as exc:
                    reason = f"worker_disconnected: {exc}"
                    self._disable(name, reason)
                    messages[name] = {"error": reason}
        for name in pending.values():
            reason = f"worker_timeout: {phase}"
            self._disable(name, reason)
            messages[name] = {"error": reason}
        return messages

    def dispatch(self, phase, jobs, *, ordered=False, deadline_error='worker_timeout'):
        """Send one phase against the active cycle with its existing wall budget."""
        if ordered:
            deadline = time.perf_counter() + self.timeout
            messages = {}
            for name, job in jobs.items():
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    messages[name] = {'error': deadline_error}
                else:
                    messages.update(self._dispatch_batch(phase, {name: job}, remaining))
            return messages
        return self._dispatch_batch(phase, jobs, self.timeout)

    def _dispatch_batch(self, phase, jobs, timeout):
        pending, messages = set(), {}
        for n, job in jobs.items():
            worker = self.workers[n]
            if worker.error:
                messages[n] = {"error": worker.error}
                continue
            try:
                worker.connection.send({"phase": phase, "cycle": self.cycle, **job})
                pending.add(n)
            except (BrokenPipeError, EOFError, OSError) as exc:
                reason = f"worker_disconnected: {exc}"
                self._disable(n, reason)
                messages[n] = {"error": reason}
        messages.update(self._collect(pending, phase, self.cycle, timeout))
        return messages

    def close(self):
        """Bounded shutdown, including workers stalled inside a native solve."""
        if self.closed:
            return
        self.closed = True
        for worker in self.workers.values():
            if not worker.error:
                try:
                    worker.connection.send({"phase": "stop", "cycle": self.cycle})
                except (BrokenPipeError, EOFError, OSError):
                    pass
        deadline = time.perf_counter() + 1.0
        for worker in self.workers.values():
            worker.process.join(timeout=max(0, deadline-time.perf_counter()))
            if worker.process.is_alive():
                worker.process.terminate()
                worker.process.join(timeout=0.1)
                if worker.process.is_alive():
                    worker.process.kill()
                    worker.process.join(timeout=0.1)
            worker.connection.close()

