#!/usr/bin/env python3
"""Run a command outside the terminal's process group so Ctrl+C cannot kill it.

With --cancel-file, SIGINT/SIGTERM/SIGHUP create that file through the
workspace mount; the mission client polls it, lands, and then exits. Without
it, these signals are ignored so landing and video finalization can complete.
"""

import argparse
from pathlib import Path
import signal
import subprocess


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cancel-file', type=Path)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('command required')

    # Register before spawning so a signal during startup is kept.
    def handle(signum, frame):
        try:
            if args.cancel_file:
                args.cancel_file.parent.mkdir(parents=True, exist_ok=True)
                args.cancel_file.touch()
                print('Stopping mission; waiting for flight cleanup and landing.', flush=True)
            else:
                print('Cleanup in progress; waiting for it to finish.', flush=True)
        except OSError:
            pass  # A closed terminal must not abort cleanup.

    previous = {sig: signal.signal(sig, handle)
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        # A terminal Ctrl+C must not kill the Docker client before cleanup ends.
        process = subprocess.Popen(command, start_new_session=True)
        result = process.wait()
        return 130 if args.cancel_file and args.cancel_file.exists() else result
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
