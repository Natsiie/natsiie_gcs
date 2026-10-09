"""Track missing frames and byte-identical frames using monotonic time."""
import hashlib


class FrameWatchdog:
    def __init__(self, read_timeout, frozen_timeout=0.0):
        self.read_timeout = read_timeout
        self.frozen_timeout = frozen_timeout
        self.reset(0.0)

    def reset(self, now):
        self.last_frame_time = now
        self.last_change_time = now
        self._signature = None
        self.repeated_frames = 0

    def observe(self, frame, now):
        self.last_frame_time = now
        if self.frozen_timeout <= 0:
            return
        # Hash every pixel; downsampling can hide real motion in a small ROI.
        signature = (frame.shape, frame.dtype.str,
                     hashlib.blake2b(frame.tobytes(), digest_size=16).digest())
        if signature != self._signature:
            self._signature = signature
            self.last_change_time = now
            self.repeated_frames = 0
        else:
            self.repeated_frames += 1

    def reason(self, now):
        if now - self.last_frame_time > self.read_timeout:
            return 'missing'
        if (self.frozen_timeout > 0 and self.repeated_frames > 0
                and now - self.last_change_time > self.frozen_timeout):
            return 'frozen'
        return None
