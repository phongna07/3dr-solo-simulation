"""Algorithm registry: each subpackage of ``core/algorithms`` exports ``ALGORITHM``.

Names are discovered without importing the packages, so settings validation
does not load solver dependencies; a package loads only when selected.
"""

import importlib
import pkgutil

from .. import algorithms
from .contracts import Algorithm


def algorithm_names():
    return tuple(sorted(module.name for module in pkgutil.iter_modules(algorithms.__path__)
                        if module.ispkg))


def algorithm_class(name):
    names = algorithm_names()
    if name not in names:
        raise ValueError(f'Unknown algorithm: {name}; use one of {", ".join(names)}')
    return importlib.import_module(f'{algorithms.__name__}.{name}').ALGORITHM


def create_algorithm(name, names, cfg, settings) -> Algorithm:
    return algorithm_class(name)(names, cfg, settings)
