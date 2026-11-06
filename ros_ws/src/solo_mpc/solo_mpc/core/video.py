"""Write received camera frames and their original simulation timestamps."""

import json
import logging
import math
import os
from pathlib import Path
import selectors
import subprocess
import time

import cv2


class _MP4Encoder:
    """Encode unchanged BGR frames, retaining a replay journal until verified.

    A successful pipe write is not an encoding acknowledgement. In particular,
    NVENC may fail after accepting frames or while flushing. Keep every input
    on disk until ffprobe has decoded and counted the finalized output.
    """

    def __init__(self, path, fps, size, timestamps_path, logger):
        self.path, self.fps, self.size = path, fps, size
        self.logger = logger
        self.work = path.with_suffix(path.suffix + '.encoding')
        self.work.mkdir()  # Never overwrite an unfinished recording.
        self.raw_path = self.work / 'frames.bgr'
        self.output = self.work / 'video.mp4'
        self.raw = self.raw_path.open('x+b')
        self.log_path = timestamps_path.with_suffix('.encoder.log')
        self.log = self.log_path.open('xb', buffering=0)
        self.process = None
        self.encoder = None
        self.frames = 0
        self.checkpoint_bytes = 0
        self.failed = False
        self.closed = False
        (self.work / 'input.json').write_text(json.dumps({
            'size': size, 'fps': fps, 'pixel_format': 'bgr24',
            'output': str(path), 'timestamps': str(timestamps_path),
        }, indent=2) + '\n')

    def _input_args(self, source):
        return ['ffmpeg', '-hide_banner', '-loglevel', 'warning', '-nostdin', '-y',
                '-filter_threads', '1', '-f', 'rawvideo', '-pixel_format', 'bgr24',
                '-video_size', f'{self.size[0]}x{self.size[1]}',
                '-framerate', str(self.fps), '-i', str(source), '-an']

    def _codec_args(self, encoder):
        if encoder == 'h264_nvenc':
            return ['-c:v', encoder, '-preset', 'p4', '-rc', 'vbr', '-cq', '18',
                    '-b:v', '0', '-pix_fmt', 'yuv420p']
        return ['-c:v', 'mpeg4', '-q:v', '2', '-threads', '2', '-pix_fmt', 'yuv420p']

    def _event(self, message, *, warning=False):
        self.log.write((message + '\n').encode())
        (self.logger.warning if warning else self.logger.info)(message)

    def _details(self):
        with self.log_path.open('rb') as stream:
            stream.seek(max(0, self.log_path.stat().st_size - 2000))
            return stream.read().decode(errors='replace').strip()

    def _select(self):
        # Exercise the actual encoder, driver, device, dimensions and pixel
        # conversion. An encoder listing or nvidia-smi alone proves none of these.
        try:
            result = subprocess.run(
                self._input_args(self.raw_path) + self._codec_args('h264_nvenc')
                + ['-frames:v', '1', '-f', 'null', '-'],
                stdout=subprocess.DEVNULL, stderr=self.log, timeout=10,
                start_new_session=True, check=False)
            if result.returncode:
                raise RuntimeError(f'NVENC probe exited {result.returncode}: {self._details()}')
        except (OSError, subprocess.TimeoutExpired, RuntimeError) as exc:
            self._event(f'Video CPU fallback: {exc}', warning=True)
            self.encoder = 'mpeg4'
        else:
            self.encoder = 'h264_nvenc'
        self._event(f'Video encoder selected: {self.encoder} ({self.path})')

    def _start(self):
        self.process = subprocess.Popen(
            self._input_args('pipe:0') + self._codec_args(self.encoder)
            + ['-fps_mode', 'passthrough', '-movflags', '+faststart', str(self.output)],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.log,
            start_new_session=True, bufsize=0)
        os.set_blocking(self.process.stdin.fileno(), False)

    def _send(self, data):
        remaining = memoryview(data)
        deadline = time.monotonic() + 10
        with selectors.DefaultSelector() as selector:
            selector.register(self.process.stdin, selectors.EVENT_WRITE)
            while remaining:
                if self.process.poll() is not None:
                    raise RuntimeError(f'encoder exited {self.process.returncode}: {self._details()}')
                wait = deadline - time.monotonic()
                if wait <= 0 or not selector.select(wait):
                    raise RuntimeError('encoder input stalled for 10 seconds')
                try:
                    count = os.write(self.process.stdin.fileno(), remaining[:1024 * 1024])
                except BlockingIOError:
                    continue
                remaining = remaining[count:]

    def _abort(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.kill()
            self.process.wait()
            self.process.stdin.close()
            self.process = None

    def _replay_cpu(self, reason):
        self._event(f'Video CPU fallback: {reason}; replaying all {self.frames} retained frames',
                    warning=True)
        self._abort()
        self.encoder = 'mpeg4'
        # Preserve the first encode until a replacement has been verified.
        self.output = self.work / 'video-cpu.mp4'
        self._event(f'Video encoder selected: {self.encoder} ({self.path})')
        self._start()
        with self.raw_path.open('rb') as stream:
            while data := stream.read(1024 * 1024):
                self._send(data)

    def write(self, frame):
        if self.failed or self.closed:
            raise RuntimeError(f'Video encoder is unavailable; input retained in {self.work}')
        data = frame.tobytes()  # No resize, filter, overlay, sampling or clock change.
        try:
            self.raw.write(data)
            self.raw.flush()
            # Bound dirty journal pages in long recordings. Otherwise many GB
            # can accumulate in the host page cache and throttle unrelated
            # optimizer captures and flight processes during bulk writeback.
            # Only the recorder waits for disk; retain every raw frame for replay.
            written = self.raw.tell()
            if written - self.checkpoint_bytes >= 32 * 1024 * 1024:
                os.fdatasync(self.raw.fileno())
                os.posix_fadvise(self.raw.fileno(), self.checkpoint_bytes,
                                written - self.checkpoint_bytes, os.POSIX_FADV_DONTNEED)
                self.checkpoint_bytes = written
            self.frames += 1
            if self.encoder is None:
                self._select()
            try:
                if self.process is None:
                    self._start()
                self._send(data)
            except (OSError, RuntimeError) as exc:
                if self.encoder != 'h264_nvenc':
                    raise
                # The journal already includes this frame, including a partial
                # pipe write; replay it once, from the beginning, on CPU.
                self._replay_cpu(str(exc))
        except BaseException:
            self.failed = True
            self._abort()
            self._event(f'Video encoding failed; input retained in {self.work}', warning=True)
            raise

    def _finish(self):
        self.process.stdin.close()
        try:
            result = self.process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            raise RuntimeError('encoder did not finalize within 15 seconds') from None
        if result:
            raise RuntimeError(f'encoder finalization exited {result}: {self._details()}')
        # Decode, rather than trusting MP4's advertised frame count. A truncated
        # bitstream or lost buffered frames must never become the final video.
        verification_timeout = max(60, 60 + 2 * self.frames / self.fps)
        self._event(f'Verifying {self.frames} encoded frames '
                    f'(timeout {verification_timeout:g}s)')
        result = subprocess.run([
            'ffprobe', '-v', 'error', '-threads', '2', '-select_streams', 'v:0', '-count_frames',
            '-show_entries', 'stream=nb_read_frames,width,height,r_frame_rate',
            '-of', 'json', str(self.output),
        ], capture_output=True, timeout=verification_timeout, check=True, start_new_session=True)
        streams = json.loads(result.stdout)['streams']
        if result.stderr or len(streams) != 1:
            raise RuntimeError(f'encoded video could not be decoded: {result.stderr.decode()}')
        stream = streams[0]
        numerator, denominator = map(int, stream['r_frame_rate'].split('/'))
        if (int(stream['nb_read_frames']) != self.frames
                or (stream['width'], stream['height']) != self.size
                or not math.isclose(numerator / denominator, self.fps, rel_tol=1e-6)):
            raise RuntimeError(f'encoded video does not match {self.frames} input frames: {stream}')

    def release(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.failed:
                raise RuntimeError(f'Video encoding failed; input retained in {self.work}')
            try:
                self._finish()
            except subprocess.TimeoutExpired:
                # Slow decoding does not establish an encoder failure. Retain
                # the original output instead of replacing it with a replay.
                raise
            except (OSError, RuntimeError, ValueError, KeyError, subprocess.SubprocessError) as exc:
                if self.encoder != 'h264_nvenc':
                    raise
                self._replay_cpu(str(exc))
                self._finish()
            self.output.replace(self.path)
            self._event(f'Video finalized: encoder={self.encoder}, frames={self.frames}, '
                        f'size={self.size}, fps={self.fps:g}, path={self.path}')
            self.raw.close()
            self.raw_path.unlink()
            for candidate in (self.work / 'video.mp4', self.work / 'video-cpu.mp4'):
                candidate.unlink(missing_ok=True)
            (self.work / 'input.json').unlink()
            self.work.rmdir()
        except BaseException:
            self._event(f'Video finalization failed; input retained in {self.work}; '
                        f'see {self.log_path}', warning=True)
            raise
        finally:
            self._abort()
            self.raw.close()
            self.log.close()


class FrameVideo:
    def __init__(self, path, fps, *, timestamps_path=None, logger=None):
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("Video FPS must be finite and positive")
        self.path = Path(path)
        self.timestamps_path = (Path(timestamps_path) if timestamps_path is not None
                                else self.path.with_suffix('.timestamps.jsonl'))
        self.codec = 'mp4v' if self.path.suffix.lower() == '.mp4' else 'MJPG'
        self.logger = logger or logging.getLogger(__name__)
        self.fps = fps
        self.writer = None
        self.timestamps = None
        self.size = None
        self.frames = 0
        self.closed = False

    def write(self, frame, stamp_ns, *, source_stamps=None):
        if self.closed:
            raise RuntimeError("Video is closed")
        size = (frame.shape[1], frame.shape[0])
        if self.writer is None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            if self.path.exists():
                raise FileExistsError(self.path)
            self.timestamps_path.parent.mkdir(parents=True, exist_ok=True)
            self.timestamps = self.timestamps_path.open("x")
            if self.codec == 'mp4v':
                self.writer = _MP4Encoder(self.path, self.fps, size,
                                          self.timestamps_path, self.logger)
            else:
                self.writer = cv2.VideoWriter(
                    str(self.path), cv2.VideoWriter_fourcc(*self.codec), self.fps, size
                )
                if not self.writer.isOpened():
                    self.close()
                    raise RuntimeError(f"Could not open {self.codec} video writer: {self.path}")
            self.size = size
        if size != self.size:
            raise ValueError(f"Camera resolution changed from {self.size} to {size}")
        record = {"frame": self.frames, "stamp_ns": stamp_ns}
        if source_stamps is not None:
            record['source_stamps_ns'] = source_stamps
        if isinstance(self.writer, _MP4Encoder):
            # Retain timing even if the encoder fails after journalling the BGR
            # frame. A disk failure is explicit and may leave a trailing record.
            self.timestamps.write(json.dumps(record) + "\n")
            self.timestamps.flush()
            self.writer.write(frame)
        else:
            self.writer.write(frame)
            self.timestamps.write(json.dumps(record) + "\n")
        self.frames += 1

    def close(self):
        try:
            if self.writer is not None:
                self.writer.release()
        finally:
            self.writer = None
            if self.timestamps is not None:
                self.timestamps.close()
                self.timestamps = None
            self.closed = True


def finalize_recording(run):
    """Finish recording before Compose starts its bounded process shutdown."""
    run = Path(run).resolve(strict=True)
    inventory = run / 'status/startup.json'
    process = json.loads(inventory.read_text())['processes'].get('combined-recorder')
    if process is None:
        return
    status_path = run / 'status/recording.json'
    work = run / 'recorded-mission.mp4.encoding'
    duration = 0
    if (work / 'input.json').exists():
        spec = json.loads((work / 'input.json').read_text())
        duration = ((work / 'frames.bgr').stat().st_size
                    / (math.prod(spec['size']) * 3 * spec['fps']))
    deadline = time.monotonic() + 300 + 4 * duration
    (run / 'status/finalize-recording.request').touch()
    print(f'Finalizing recording in {run}; waiting for verified output', flush=True)
    while True:
        status = json.loads(status_path.read_text()) if status_path.exists() else {}
        state = status.get('state')
        if state == 'finalized':
            if not (run / 'recorded-mission.mp4').is_file() or work.exists():
                raise RuntimeError('Recorder reported success without a finalized MP4')
            print(f"Video saved: {run / 'recorded-mission.mp4'}", flush=True)
            return
        if state == 'empty':
            print('No airborne camera frames were recorded', flush=True)
            return
        if state == 'failed':
            raise RuntimeError(f"Video finalization failed: {status.get('detail')}")
        process = json.loads(inventory.read_text())['processes']['combined-recorder']
        if process['state'] == 'exited':
            raise RuntimeError(f"Recorder exited {process['exit_code']} without finalizing video")
        if time.monotonic() >= deadline:
            raise RuntimeError(f'Video finalization timed out; input retained in {work}')
        time.sleep(0.5)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Finalize recording before stack shutdown')
    parser.add_argument('run')
    finalize_recording(parser.parse_args().run)
