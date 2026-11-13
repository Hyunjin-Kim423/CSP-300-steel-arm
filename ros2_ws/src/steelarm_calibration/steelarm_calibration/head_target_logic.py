"""Pure decision logic for head_target_node (no ROS imports) so it can be unit-tested."""
import math


def evaluate_sample(translation, age_s, max_age_s):
    """Decide whether a looked-up TF translation may be published as a target.

    translation: (x, y, z) or None when the lookup failed. Returns (valid, reason).
    """
    if translation is None:
        return False, 'tf unavailable'
    if not all(math.isfinite(v) for v in translation):
        return False, 'non-finite translation'
    if age_s > max_age_s:
        return False, f'tf stale ({age_s:.2f}s > {max_age_s:.2f}s)'
    return True, 'ok'


class ArrivalAgeTracker:
    """Age of the newest sample measured on the local clock only.

    The sample's own stamp is only compared with the previous stamp (changed = a new sample arrived),
    never with local time, so a sender whose clock is off by seconds is still judged correctly.
    """

    def __init__(self):
        self.stamp_ns = None
        self.arrived_ns = None

    def update(self, stamp_ns, now_ns):
        if stamp_ns != self.stamp_ns:
            self.stamp_ns = stamp_ns
            self.arrived_ns = now_ns
        return (now_ns - self.arrived_ns) / 1e9

    def reset(self):
        self.stamp_ns = None
        self.arrived_ns = None


class ValidityTracker:
    """Reports a change only when the valid/invalid state flips (the first sample counts as a change)."""

    def __init__(self):
        self.valid = None

    def update(self, valid):
        changed = valid != self.valid
        self.valid = valid
        return changed
