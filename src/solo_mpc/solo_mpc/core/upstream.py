"""Load unchanged upstream modules without importing its simulator/config.

Upstream is a flat script repository. Its sibling imports (``import dynamics``)
are bound privately to the declared modules next to it, without changing
sys.path, sys.modules['dynamics'], or the process-wide importer. Modules load
on first use, so each algorithm loads and hashes only its own sources. Spawned
UAV workers load their own module instances.
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
            if ancestor / 'src/solo_mpc/solo_mpc/core' == core:
                directory = ancestor / 'external/motion_planning_MPC'
                break
            candidate = ancestor / 'share/solo_mpc/upstream'
            if candidate.is_dir():
                directory = candidate
                break
        else:
            raise ImportError('Cannot locate MPC upstream sources; rebuild the solo_mpc package')
    return directory


SOURCE_DIRECTORY = _source_directory()
SOURCE_HASHES = {}
# Upstream module -> sibling modules its source imports by bare name.
IMPORTS = {'dynamics': (), 'mpc_problem': ('dynamics',), 'apfc': ('dynamics',),
           'apf': ('apfc',), 'cbfqp': ('apfc',)}


def _load(name, imports=None):
    qualified = f'{__package__}._upstream_{name}'
    if not (SOURCE_DIRECTORY / f'{name}.py').is_file():
        raise ImportError(
            f'Missing upstream {SOURCE_DIRECTORY / name}.py. Run '
            'git submodule update --init --recursive in 3dr-solo-simulation, '
            'then rebuild the image if using installed code.')
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


@lru_cache(maxsize=None)
def load(name):
    """One private instance per process of an upstream module and its imports."""
    if name not in IMPORTS:
        raise ImportError(f'Upstream module {name!r} is not declared in core/upstream.py')
    return _load(name, {dependency: load(dependency) for dependency in IMPORTS[name]})


@lru_cache(maxsize=None)
def provenance(modules):
    """Load, then report actual source hashes of exactly ``modules`` (a tuple)."""
    for name in modules:
        load(name)
    metadata = Path(__file__).resolve().parents[2] / 'UPSTREAM.json'
    if not metadata.is_file():
        metadata = SOURCE_DIRECTORY.parent / 'UPSTREAM.json'
    declared = json.loads(metadata.read_text())
    hashes = {f'{name}.py': SOURCE_HASHES[f'{name}.py'] for name in modules}
    return dict(repository=declared['repository'], pinned_commit=declared['commit'],
                source_directory=str(SOURCE_DIRECTORY), sha256=hashes,
                matches_pinned_sources=all(
                    declared['sha256'].get(name) == digest for name, digest in hashes.items()))
