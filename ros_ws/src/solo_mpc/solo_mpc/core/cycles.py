"""One outstanding fleet computation; callbacks own inputs, workers own planning."""

from concurrent.futures import ThreadPoolExecutor
import copy
import time


class CycleRunner:
    def __init__(self):
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='fleet-cycle')
        self.future = None
        self.started = None
        self.completed = None
        self.collected = None
        self.closed = False

    def submit(self, function, *args, **kwargs):
        if self.closed or self.future is not None:
            raise RuntimeError('A fleet cycle is already active or closed')
        # Callbacks can replace/mutate live messages immediately after submission.
        args, kwargs = copy.deepcopy((args, kwargs))
        self.started = time.monotonic()
        self.completed = self.collected = None

        def compute():
            try:
                return function(*args, **kwargs)
            finally:
                # Record before the future becomes done, including exceptions.
                self.completed = time.monotonic()

        self.future = self.executor.submit(compute)

    def take(self):
        if self.future is None or not self.future.done():
            return None
        future, self.future = self.future, None
        self.collected = time.monotonic()
        return future.result()

    def timing(self):
        return {
            'fleet_started_wall': self.started,
            'fleet_completed_wall': self.completed,
            'fleet_collected_wall': self.collected,
            'completion_to_collection_ms': 1000 * (self.collected - self.completed),
        }

    def close(self):
        self.closed = True
        # WorkerPool enforces phase deadlines and kills stalled
        # native workers, so the coordinator is joined before its pipes close.
        self.executor.shutdown(wait=True, cancel_futures=True)
        self.future = None


def result_is_current(source_stamp, now, started_wall, settings):
    age = now - source_stamp
    return (0 <= age <= settings['command_timeout_sim']
            and time.monotonic() - started_wall <= settings['command_timeout_wall'])
