"""Offline regressions for RGB-D continuity; does not open a physical camera."""
import csv
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from sequence_capture import Recording, parse_args


class RecordingContinuityTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="rgbd-regression-")
        self.recordings = []

    def tearDown(self):
        for recording in self.recordings:
            if recording.thread.is_alive():
                recording.finish()
        self.tmp.cleanup()

    def recording(self, frames=90, mode="depth"):
        args = SimpleNamespace(output=Path(self.tmp.name), mode=mode, label="regression",
                               fps=30, width=32, height=24, frames=frames, seconds=120)
        r = Recording(cv2, args, {"depth_unit_m": 0.001}, {})
        self.recordings.append(r)
        return r

    def frame(self, color_id, depth_id=None):
        if depth_id is None:
            depth_id = color_id
        return {
            "color": np.full((24, 32, 3), color_id % 255, np.uint8),
            "depth_raw": np.full((24, 32), 1000 + depth_id, np.uint16),
            "depth_aligned": np.full((24, 32), 1000 + depth_id, np.uint16),
            "color_frame_number": color_id, "depth_frame_number": depth_id,
            "color_timestamp_ms": color_id * 1000 / 30,
            "depth_timestamp_ms": depth_id * 1000 / 30,
            "host_monotonic_s": time.monotonic(),
        }

    def offer(self, recording, color_id, depth_id=None):
        recording.offer(self.frame(color_id, depth_id))
        # Keep this continuity regression independent of disk speed/queue overflow.
        deadline = time.monotonic() + 5
        while recording.accepted - recording.written > 8:
            if recording.error or time.monotonic() > deadline:
                self.fail(recording.error or "writer timed out")
            time.sleep(0.001)

    def test_duplicate_at_66_does_not_stop_90_frame_burst(self):
        r = self.recording()
        for n in range(1, 67):
            self.offer(r, n)
        self.offer(r, 66)
        self.assertFalse(r.done.is_set())
        self.assertEqual(r.accepted, 66)
        for n in range(67, 91):
            self.offer(r, n)
        info = r.finish()
        self.assertTrue(info["complete"])
        self.assertEqual(info["saved_frames"], 90)
        self.assertEqual(info["skipped_duplicate_pairs"], 1)
        self.assertTrue(info["consecutive_frames"])
        with (r.path / "frames.csv").open() as f:
            ids = [int(row["depth_frame_number"]) for row in csv.DictReader(f)]
        self.assertEqual(ids, list(range(1, 91)))
        depth = cv2.imread(str(r.path / "depth_raw/000089.png"), cv2.IMREAD_UNCHANGED)
        self.assertEqual(depth.dtype, np.uint16)
        self.assertTrue((depth == 1090).all())

    def test_reused_depth_and_color_are_skipped_and_gaps_counted_once(self):
        r = self.recording(frames=3)
        for color, depth in [(1, 1), (2, 1), (3, 1), (4, 2), (4, 3), (5, 4)]:
            self.offer(r, color, depth)
        info = r.finish()
        self.assertTrue(info["complete"])
        self.assertEqual(info["saved_frames"], 3)
        self.assertEqual(info["skipped_duplicate_pairs"], 3)
        self.assertEqual(info["reused_depth_frames"], 2)
        self.assertEqual(info["reused_color_frames"], 1)
        self.assertEqual(info["color_frame_gaps"], 2)
        self.assertEqual(info["depth_frame_gaps"], 1)
        self.assertFalse(info["consecutive_frames"])

    def test_actual_frame_reversal_is_an_error_not_a_complete_recording(self):
        r = self.recording()
        self.offer(r, 10)
        self.offer(r, 9)
        info = r.finish()
        self.assertEqual(info["stop_reason"], "frame_number_error")
        self.assertFalse(info["complete"])
        self.assertFalse(info["consecutive_frames"])
        self.assertTrue(info["frame_number_reversed"])

    def test_unlimited_capture_continues_past_90_until_user_stops(self):
        r = self.recording(frames=0)
        for n in range(1, 101):
            self.offer(r, n)
        self.assertFalse(r.done.is_set())
        info = r.finish()
        self.assertTrue(info["complete"])
        self.assertEqual(info["stop_reason"], "user_stop")
        self.assertEqual(info["saved_frames"], 100)
        self.assertEqual(json.loads(r.info_path.read_text())["requested_frames"], 0)

    def test_early_stop_of_finite_capture_stays_incomplete(self):
        r = self.recording(frames=90)
        self.offer(r, 1)
        self.assertFalse(r.finish()["complete"])

    def test_video_also_skips_duplicates_and_avi_contains_unique_samples(self):
        r = self.recording(mode="video")
        for n in (1, 1, 2, 3):
            self.offer(r, n)
        r.stop("duration_reached")
        info = r.finish()
        self.assertTrue(info["complete"])
        self.assertEqual(info["saved_frames"], 3)
        cap = cv2.VideoCapture(str(r.path / "color.avi"))
        count = 0
        while cap.read()[0]:
            count += 1
        cap.release()
        self.assertEqual(count, 3)

    def test_depth_defaults_to_continuous_and_accepts_explicit_burst(self):
        with patch("sys.argv", ["capture_depth_sequence.py"]):
            self.assertEqual(parse_args("depth").frames, 0)
        with patch("sys.argv", ["capture_depth_sequence.py", "--frames", "90"]):
            self.assertEqual(parse_args("depth").frames, 90)


if __name__ == "__main__":
    unittest.main()
