"""Configuration validation shared by world generation and mission launch."""
import math
import re
from pathlib import Path

import yaml


def load_config(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('Expected schema_version: 1')
    uavs = data.get('uavs')
    if not isinstance(uavs, list) or not uavs:
        raise ValueError('uavs must be a nonempty list')
    names = set()
    for uav in uavs:
        if not isinstance(uav, dict):
            raise ValueError('Each UAV must be a mapping')
        name = uav.get('name', '')
        if not isinstance(name, str) or not re.fullmatch(r'[a-z][a-z0-9_]*', name) or name in names:
            raise ValueError(f'Invalid/duplicate UAV name: {name}')
        names.add(name)
        spawn = uav.get('spawn')
        if not isinstance(spawn, list) or len(spawn) != 6 or not all(
            type(x) in (float, int) and math.isfinite(x) for x in spawn
        ):
            raise ValueError(f'{name}: spawn must have six finite values (degrees)')
    world = data.get('world_name', '')
    if not isinstance(world, str) or not re.fullmatch(r'[a-z][a-z0-9_]*', world):
        raise ValueError('Invalid world_name')
    return data
