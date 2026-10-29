"""Load the unchanged upstream solver without importing its simulator/config.

Upstream is a flat script repository. Its ``import dynamics`` is bound privately
to the dynamics next to the solver, without changing sys.path, sys.modules['dynamics'],
or the process-wide importer. Spawned UAV workers load their own module instances.
"""

import builtins
from functools import lru_cache
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys


def _source_directory():
    override = os.environ.get('SOLO_MPC_UPSTREAM_PATH')
    if override:
        directory = Path(override).resolve()
    else:
        core = Path(__file__).resolve().parent
        for ancestor in core.parents:
            if ancestor / 'ros_ws/src/solo_mpc/solo_mpc/core' == core:
                directory = ancestor / 'external/motion_planning_MPC'
                break
            candidate = ancestor / 'share/solo_mpc/upstream'
            if candidate.is_dir():
                directory = candidate
                break
        else:
            raise ImportError('Cannot locate MPC upstream sources; rebuild the solo_mpc package')
    for name in ('mpc_problem.py', 'dynamics.py'):
        if not (directory / name).is_file():
            raise ImportError(
                f'Missing upstream {directory / name}. Run '
                'git submodule update --init --recursive in 3dr-solo-simulation, '
                'then rebuild the image if using installed code.')
    return directory


SOURCE_DIRECTORY = _source_directory()
SOURCE_HASHES = {}


def _load(name, imports=None):
    qualified = f'{__package__}._upstream_{name}'
    spec = importlib.util.spec_from_file_location(qualified, SOURCE_DIRECTORY / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    if imports:
        def upstream_import(name, globals=None, locals=None, fromlist=(), level=0):
            if level == 0 and name in imports:
                return imports[name]
            return builtins.__import__(name, globals, locals, fromlist, level)
        module.__dict__['__builtins__'] = {**vars(builtins), '__import__': upstream_import}
    sys.modules[qualified] = module
    try:
        source = Path(spec.origin).read_bytes()
        # Upstream tracks some bytecode; execute the actual source, never a
        # possibly stale .pyc, and do not write anything into the submodule.
        exec(compile(source, spec.origin, 'exec'), module.__dict__)
        SOURCE_HASHES[f'{name}.py'] = hashlib.sha256(source).hexdigest()
    except BaseException:
        sys.modules.pop(qualified, None)
        raise
    return module


dynamics = _load('dynamics')
solver = _load('mpc_problem', {'dynamics': dynamics})


@lru_cache(maxsize=1)
def provenance():
    """Report actual loaded source hashes, alongside the recorded source pin."""
    metadata = Path(__file__).resolve().parents[2] / 'UPSTREAM.json'
    if not metadata.is_file():
        metadata = SOURCE_DIRECTORY.parent / 'UPSTREAM.json'
    declared = json.loads(metadata.read_text())
    hashes = dict(SOURCE_HASHES)
    return dict(repository=declared['repository'], pinned_commit=declared['commit'],
                source_directory=str(SOURCE_DIRECTORY), sha256=hashes,
                matches_pinned_sources=all(
                    declared['sha256'].get(name) == digest for name, digest in hashes.items()))
