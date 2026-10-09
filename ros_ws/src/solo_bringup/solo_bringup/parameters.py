"""Apply plugin YAML through services: pinned MAVROS disables global arguments on plugins."""
from pathlib import Path
import time

from rcl_interfaces.srv import ListParameters, SetParameters
from rclpy.parameter import Parameter
import yaml


def flatten_parameters(values, prefix=''):
    for name, value in values.items():
        key = prefix + name
        if isinstance(value, dict):
            yield from flatten_parameters(value, key + '.')
        else:
            yield key, value


class PluginParameters:
    def __init__(self, node, uavs, run_dir):
        self.node = node
        self.items = {}
        self.required = set()
        for uav in uavs:
            prefix = f"/{uav['name']}/mavros/"
            self.required.update(prefix + plugin for plugin in ('local_position', 'imu', 'sys'))
            data = yaml.safe_load((Path(run_dir) / f"{uav['name']}-mavros.yaml").read_text())
            for selector, config in data.items():
                if not selector.startswith('/**/'):
                    continue
                target = prefix + selector.removeprefix('/**/')
                values = config['ros__parameters']
                self.items[target] = {
                    'client': node.create_client(SetParameters, target + '/set_parameters'),
                    'list_client': node.create_client(ListParameters, target + '/list_parameters'),
                    'request': SetParameters.Request(parameters=[
                        Parameter(key, value=value).to_parameter_msg()
                        for key, value in flatten_parameters(values)
                    ]),
                    'future': None, 'done': False, 'at': 0, 'phase': 'list',
                }

    def tick(self):
        now = time.monotonic()
        active = {f"{namespace.rstrip('/')}/{name}" for name, namespace in
                  self.node.get_node_names_and_namespaces()}
        for target, item in self.items.items():
            if item['done'] or target not in active:
                continue
            future = item['future']
            if future is not None:
                if future.done():
                    response = future.result()
                    item['future'] = None
                    if response is None:
                        raise RuntimeError(f'{target}: no plugin parameter response')
                    if item['phase'] == 'list':
                        declared = set(response.result.names)
                        parameters = item['request'].parameters
                        obsolete = [p.name for p in parameters if p.name not in declared]
                        if target.endswith(('/local_position', '/imu')) and 'frame_id' not in declared:
                            raise RuntimeError(f'{target}: required frame_id parameter missing')
                        if obsolete:
                            self.node.get_logger().warning(f'{target}: upstream YAML has undeclared keys; ignored: {obsolete}')
                        item['request'].parameters = [p for p in parameters if p.name in declared]
                        item['phase'] = 'set'
                    else:
                        if not all(result.successful for result in response.results):
                            raise RuntimeError(f'{target}: plugin parameters rejected: {response}')
                        item['done'] = True
                        self.node.get_logger().info(f'{target}: plugin parameters applied')
                elif now - item['at'] > 20:
                    future.cancel()
                    item['future'] = None
            elif item['client'].service_is_ready() and item['list_client'].service_is_ready():
                item['future'] = item['list_client'].call_async(ListParameters.Request(depth=0)) \
                    if item['phase'] == 'list' else item['client'].call_async(item['request'])
                item['at'] = now
        expected = (active & self.items.keys()) | self.required
        return all(self.items[target]['done'] for target in expected)
