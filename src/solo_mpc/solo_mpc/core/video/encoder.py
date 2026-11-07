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


def _compact_codec(fps):
    """Long-GOP CPU settings for the published file; live segments stay short."""
    return ['-c:v', 'libx264', '-preset', 'medium', '-crf', '23', '-tune', 'animation',
            '-g', str(max(1, round(10 * fps))), '-threads', '2', '-pix_fmt', 'yuv420p']


class _MP4Encoder:
    """Stream compressed one-second segments; retry only a bounded RAM window."""

    def __init__(self, path, fps, size, timestamps_path, logger):
        self.path, self.fps, self.size = path, fps, size
        self.timestamps_path, self.logger = timestamps_path, logger
        self.work = path.with_suffix(path.suffix + '.encoding')
        self.work.mkdir()
        self.log_path = timestamps_path.with_suffix('.encoder.log')
        self.log = self.log_path.open('ab', buffering=0)
        self.process = None
        self.encoder = 'h264_nvenc'
        self.frames = 0
        self.buffer = []
        self.segment = 0
        self.closed = False
        self.summary = None
        self.fallbacks = []
        self._save_spec()

    def _save_spec(self):
        target = self.work / 'input.json'
        temporary = target.with_suffix('.tmp')
        temporary.write_text(json.dumps({
            'size': self.size, 'fps': self.fps, 'frames': self.frames,
            'timestamps': os.path.relpath(self.timestamps_path, self.path.parent),
            'fallbacks': self.fallbacks,
        }) + '\n')
        temporary.replace(target)

    def _event(self, message, *, warning=False):
        self.log.write((message + '\n').encode())
        (self.logger.warning if warning else self.logger.info)(message)

    def _start(self):
        self.output = self.work / f'{self.segment:08d}-{self.encoder}.mp4'
        codec = (['-c:v', 'h264_nvenc', '-preset', 'p4', '-rc', 'vbr', '-cq', '18',
                  '-b:v', '0'] if self.encoder == 'h264_nvenc' else
                 ['-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '20', '-threads', '2'])
        self.process = subprocess.Popen([
            'ffmpeg', '-hide_banner', '-loglevel', 'warning', '-nostdin', '-y',
            '-filter_threads', '1', '-f', 'rawvideo', '-pixel_format', 'bgr24',
            '-video_size', f'{self.size[0]}x{self.size[1]}', '-framerate', str(self.fps),
            '-i', 'pipe:0', '-an', *codec, '-pix_fmt', 'yuv420p', '-bf', '0',
            '-g', str(max(1, round(self.fps / 4))), '-fps_mode', 'passthrough',
            '-movflags', '+frag_keyframe+empty_moov+default_base_moof',
            '-video_track_timescale', '90000', '-flush_packets', '1', str(self.output)],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.log,
            start_new_session=True, bufsize=0)
        os.set_blocking(self.process.stdin.fileno(), False)
        (self.output.with_suffix('.json')).write_text(json.dumps({
            'start': self.frames - len(self.buffer), 'count': len(self.buffer),
        }))

    def _send(self, data):
        remaining = memoryview(data)
        deadline = time.monotonic() + 10
        with selectors.DefaultSelector() as selector:
            selector.register(self.process.stdin, selectors.EVENT_WRITE)
            while remaining:
                if self.process.poll() is not None:
                    raise RuntimeError(f'encoder exited {self.process.returncode}')
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

    def _finish(self):
        self.process.stdin.close()
        code = self.process.wait(timeout=15)
        if code:
            raise RuntimeError(f'encoder flush exited {code}')
        self.process = None
        if _decoded_frames(self.output, self.size, self.fps) != len(self.buffer):
            raise RuntimeError('segment decoded frame count differs from submitted count')

    def _failure(self, exc):
        self._abort()
        if self.encoder == 'h264_nvenc':
            message = f'Video CPU fallback at input frame {self.frames}: {exc}'
            self.fallbacks.append(message)
            self._event(message, warning=True)
            self.encoder = 'libx264'
            self._save_spec()
            try:
                self._start()
                for data in self.buffer:
                    self._send(data)
                return
            except (OSError, RuntimeError, subprocess.SubprocessError) as retry:
                self._abort()
                self._event(f'Video CPU retry failed: {retry}', warning=True)
        self._event(f'Video segment interrupted: {exc}; retaining decodable prefix; '
                    'missing frames will be reported', warning=True)
        self.buffer.clear()
        self.segment += 1

    def write(self, frame):
        if self.closed:
            raise RuntimeError('Video is closed')
        self.buffer.append(frame.tobytes())
        self.frames += 1
        self._save_spec()
        try:
            if self.process is None:
                self._start()
            # Update the metadata before submitting: a killed recorder may leave
            # only a decodable prefix of this attempt.
            self.output.with_suffix('.json').write_text(json.dumps({
                'start': self.frames - len(self.buffer), 'count': len(self.buffer),
            }))
            self._send(self.buffer[-1])
        except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
            self._failure(exc)
        if len(self.buffer) >= max(1, round(self.fps)):
            self._seal()

    def _seal(self):
        if not self.buffer:
            return
        try:
            self._finish()
        except (OSError, RuntimeError, ValueError, KeyError, subprocess.SubprocessError) as exc:
            self._failure(exc)
            if self.buffer:
                try:
                    self._finish()
                except (OSError, RuntimeError, ValueError, KeyError,
                        subprocess.SubprocessError) as retry:
                    self._abort()
                    self._event(f'Video segment flush failed: {retry}', warning=True)
        self.buffer.clear()
        self.segment += 1

    def release(self):
        if self.closed:
            return
        self.closed = True
        try:
            self._seal()
            self.summary = recover_video(self.path, self.logger)
        finally:
            self._abort()
            self.log.close()


def _decoded_frames(path, size, fps, *, timeout=60):
    result = subprocess.run([
        'ffprobe', '-v', 'error', '-threads', '2', '-select_streams', 'v:0',
        '-count_frames', '-show_entries', 'stream=nb_read_frames,width,height,r_frame_rate,avg_frame_rate',
        '-of', 'json', str(path)], capture_output=True, timeout=timeout, check=True)
    if result.stderr:
        raise RuntimeError(f'Video decode errors: {result.stderr.decode(errors="replace")}')
    stream, = json.loads(result.stdout)['streams']
    # FFprobe can advertise twice the nominal r_frame_rate for a single
    # fragmented sample. Its average still reflects the actual sample duration.
    rate = stream['avg_frame_rate' if int(stream.get('nb_read_frames', 0)) == 1
                  else 'r_frame_rate'].split('/')
    if ((stream['width'], stream['height']) != tuple(size)
            or not math.isclose(int(rate[0]) / int(rate[1]), fps, rel_tol=1e-6)):
        raise RuntimeError(f'Unexpected video format: {stream}')
    return int(stream.get('nb_read_frames', 0))


def recover_video(path, logger=None, *, interrupted=False):
    """Recover compressed prefixes only; never reconstruct or save raw frames."""
    path = Path(path)
    work = path.with_suffix(path.suffix + '.encoding')
    spec = json.loads((work / 'input.json').read_text())
    fps, size = spec['fps'], spec['size']
    timestamps = path.parent / spec['timestamps']
    inputs = timestamps.with_suffix('.inputs.jsonl')
    if not inputs.exists():
        # Preserve the original source-time journal, including failed inputs.
        timestamps.replace(inputs)
    records = []
    lines = inputs.read_text().splitlines()
    for index, line in enumerate(lines):
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError:
            if index != len(lines) - 1:
                raise
            interrupted = True  # A kill may interrupt the final journal write.
    total = len(records)
    retained = {}
    segments = []
    log_path = timestamps.with_suffix('.encoder.log')
    with log_path.open('ab', buffering=0) as log:
        # CPU retries supersede GPU attempts only where they actually decoded.
        for source in sorted(work.glob('[0-9]*.mp4')):
            if source.name.endswith('.recovered.mp4'):
                continue
            try:
                meta = json.loads(source.with_suffix('.json').read_text())
                target = source.with_suffix('.recovered.mp4')
                subprocess.run(['ffmpeg', '-v', 'warning', '-nostdin', '-y',
                                '-threads', '2', '-i', str(source), '-map', '0:v:0',
                                '-c', 'copy', '-movflags', '+faststart', str(target)],
                               stdout=subprocess.DEVNULL, stderr=log, timeout=30, check=True)
                count = min(_decoded_frames(target, size, fps), meta['count'],
                            total - meta['start'])
                if count <= 0:
                    continue
                segments.append((meta['start'], count, target))
            except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as exc:
                log.write(f'Unusable segment {source.name}: {exc}\n'.encode())
        # Select the longest prefix for each segment start (retry can be shorter).
        selected = {}
        for start, count, target in segments:
            if start not in selected or count > selected[start][0]:
                selected[start] = (count, target)
        for start, (count, _) in selected.items():
            for index in range(start, start + count):
                retained[index] = True
        if not retained:
            raise RuntimeError(f'No decodable video frames; compressed evidence retained in {work}')
        missing = [i for i in range(total) if i not in retained]
        gaps = []
        for i in missing:
            if gaps and gaps[-1]['end_frame'] == i:
                gaps[-1]['end_frame'] = i + 1
            else:
                gaps.append({'start_frame': i, 'end_frame': i + 1})
        starts = sorted(selected)
        manifest = work / 'concat.txt'
        manifest.write_text(''.join(
            f"file '{selected[start][1].name}'\n" +
            f"duration {((starts[n + 1] if n + 1 < len(starts) else total) - start) / fps:.9f}\n"
            for n, start in enumerate(starts)))
        output = work / 'final.mp4'
        command = ['ffmpeg', '-v', 'warning', '-nostdin', '-y', '-filter_threads', '1',
                   '-threads', '2', '-f', 'concat', '-safe', '0', '-i', str(manifest)]
        if missing:
            # Hold the nearest retained image across holes, without shortening
            # simulation time. These are explicitly marked in the sidecar.
            command += ['-vf', f'tpad=start_mode=clone:start_duration={starts[0] / fps}:stop_mode=clone:'
                        f'stop_duration={total / fps},fps={fps}',
                        '-frames:v', str(total), *_compact_codec(fps)]
        else:
            command += ['-c', 'copy']
        subprocess.run(command + ['-movflags', '+faststart', str(output)],
                       stdout=subprocess.DEVNULL, stderr=log,
                       timeout=300 + 4 * total / fps, check=True)
        if _decoded_frames(output, size, fps, timeout=60 + 2 * total / fps) != total:
            raise RuntimeError('Recovered output frame count mismatch; compressed inputs retained')
        compression = 'libx264 medium crf 23'
        if not missing:
            # Joined live segments keep their short GOPs; recompress once and
            # publish the verified joined copy if that pass fails.
            compact = work / 'compact.mp4'
            try:
                subprocess.run(['ffmpeg', '-v', 'warning', '-nostdin', '-y', '-threads', '2',
                                '-i', str(output), '-map', '0:v:0', '-an', *_compact_codec(fps),
                                '-fps_mode', 'passthrough', '-movflags', '+faststart',
                                str(compact)], stdout=subprocess.DEVNULL, stderr=log,
                               timeout=300 + 4 * total / fps, check=True)
                if _decoded_frames(compact, size, fps, timeout=60 + 2 * total / fps) != total:
                    raise RuntimeError('compressed frame count mismatch')
                output.unlink()
                output = compact
            except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as exc:
                compression = f'skipped, joined live segments published: {exc}'
                log.write(f'Video compression {compression}\n'.encode())
                if logger:
                    logger.warning(f'Video compression {compression}')
        summary = {'state': 'partial' if missing or interrupted else 'finalized',
                   'input_frames': total, 'retained_frames': len(retained),
                   'dropped_frames': len(missing), 'held_frames': len(missing),
                   'gaps': gaps, 'fallbacks': spec['fallbacks'], 'interrupted': interrupted,
                   'fps': fps, 'size': size, 'compression': compression}
        final_stamps = timestamps.with_suffix('.tmp')
        previous = min(retained)
        with final_stamps.open('w') as stream:
            for i, record in enumerate(records):
                if i in retained:
                    previous = i
                item = dict(records[previous])
                item.update(frame=i, stamp_ns=record['stamp_ns'],
                            playback_ns=round(i * 1_000_000_000 / fps),
                            held=i not in retained, source_frame=previous)
                stream.write(json.dumps(item) + '\n')
        final_stamps.replace(timestamps)
        report = timestamps.with_suffix('.recovery.json')
        report.write_text(json.dumps(summary, indent=2) + '\n')
        output.replace(path)
        message = f'Video {summary["state"]}: {json.dumps(summary)}; output={path}'
        log.write((message + '\n').encode())
        if logger:
            (logger.warning if summary['state'] == 'partial' else logger.info)(message)
        # Compressed segments stay available for audit and interrupted recovery.
        return summary


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
        self.summary = None

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
            # Journal source times only; raw pixels never reach disk.
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
                self.summary = getattr(self.writer, 'summary', None)
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
        duration = spec['frames'] / spec['fps']
    deadline = time.monotonic() + 300 + 4 * duration
    (run / 'status/finalize-recording.request').touch()
    print(f'Finalizing recording in {run}; waiting for verified output', flush=True)
    while True:
        status = json.loads(status_path.read_text()) if status_path.exists() else {}
        state = status.get('state')
        if state in ('finalized', 'partial'):
            if not (run / 'recorded-mission.mp4').is_file():
                raise RuntimeError('Recorder reported success without a finalized MP4')
            print(f"Video {state}: {run / 'recorded-mission.mp4'}; {status.get('detail', '')}", flush=True)
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


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Finalize recording before stack shutdown')
    parser.add_argument('run')
    parser.add_argument('--recover', action='store_true',
                        help='Recover compressed segments after recorder/stack has stopped')
    args = parser.parse_args()
    if args.recover:
        run = Path(args.run).resolve()
        summary = recover_video(run / 'recorded-mission.mp4', interrupted=True)
        status = run / 'status/recording.json'
        status.parent.mkdir(exist_ok=True)
        status.write_text(json.dumps({
            'state': 'partial', 'detail': json.dumps(summary),
            'updated_wall_unix': time.time(),
        }) + '\n')
        print(json.dumps(summary, indent=2))
    else:
        finalize_recording(args.run)


if __name__ == '__main__':
    main()
