import math
import unittest

import numpy as np

from boundary import inside
from exact import preset_boundary
from medial_axis import analyze


def blob(seed, depth=.8):
    rng = np.random.default_rng(seed)
    t = np.arange(160) / 160 * 2 * np.pi
    phases, amps = rng.random(7) * 2 * np.pi, (rng.random(7) * .6 + .4) / np.arange(2, 9) ** 1.3
    r = 160 + sum(95 * depth * amps[k] * np.cos((k + 2) * t + phases[k]) for k in range(7))
    return np.c_[300 + r * np.cos(t), 300 + r * np.sin(t)]


class MedialAxisTests(unittest.TestCase):
    def test_every_preset_and_random_blob_succeeds(self):
        shapes = [preset_boundary({"kind": k, "depth": .55, "arms": 5})
                  for k in ("circle", "ellipse", "star", "flower", "crescent", "neck")]
        shapes += [blob(seed) for seed in range(20)]
        for polygon in shapes:
            result = analyze(polygon.tolist())
            path = np.asarray(result["path"])
            self.assertGreater(result["stats"]["length"], 50)
            self.assertTrue(all(inside(p, polygon) for p in path[::10]))

    def test_ellipse_trunk_is_major_axis(self):
        result = analyze(preset_boundary({"kind": "ellipse"}).tolist())
        path = np.asarray(result["path"])
        self.assertLess(np.max(np.abs(path[:, 1] - 300)), 1.)
        self.assertAlmostEqual(result["stats"]["length"], 2 * 205 * 1.12, delta=2)

    def test_crescent_follows_mid_radius(self):
        result = analyze(preset_boundary({"kind": "crescent", "depth": .55}).tolist(), extend=False)
        radius = np.linalg.norm(np.asarray(result["path"]) - 300, axis=1)
        self.assertLess(np.max(np.abs(radius[50:-50] - (220 + 120.5) / 2)), 2.)

    def test_progress_is_monotone_and_complete(self):
        events = []
        analyze(blob(3).tolist(), progress=lambda f, m: events.append(f))
        self.assertEqual(events[-1], 1.)
        self.assertEqual(events, sorted(events))

    def test_invalid_input(self):
        for kwargs in ({"points": [[100, 100], [500, 500], [100, 500], [500, 100]]},
                       {"points": [[math.nan, 100], [200, 300], [300, 100]]},
                       {"points": preset_boundary({"kind": "circle"}).tolist(), "prune": 2}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                analyze(**kwargs)


class ComplexityTests(unittest.TestCase):
    def score(self, polygon):
        from boundary import validate_polygon
        from complexity import describe
        return describe(validate_polygon(polygon), analyze(polygon.tolist()))

    def test_ordering_and_necks(self):
        circle = self.score(preset_boundary({"kind": "circle"}))
        star = self.score(preset_boundary({"kind": "star", "depth": .55, "arms": 5}))
        neck = self.score(preset_boundary({"kind": "neck", "depth": .8}))
        self.assertEqual(circle["score"], 0)
        self.assertGreater(star["score"], 25)
        self.assertEqual(neck["necks"], 1)
        self.assertEqual(star["reflexVertices"], 5)

    def test_lanes_partition_field(self):
        from sweep import compare
        result = compare(blob(2).tolist(), drones=8)
        for name, method in result["methods"].items():
            lanes = np.asarray(method["lanes"]["values"])
            self.assertEqual(set(np.unique(lanes[lanes >= 0])), set(range(8)), name)
            self.assertLess(np.count_nonzero(lanes == -1), 20, name)
            self.assertEqual(len(method["drones"]), 8)

    def test_nadir_images_every_cell(self):
        from sweep import compare
        for polygon in (blob(5), preset_boundary({"kind": "star", "depth": .55, "arms": 5})):
            result = compare(polygon.tolist(), drones=6, hfov=60, vfov=45, overlap=.1)
            for name, method in result["methods"].items():
                self.assertEqual(method["stats"]["gapCells"], 0, name)
                self.assertEqual(method["stats"]["cameraCoverage"], 100.0, name)
                imaged = np.asarray(method["imaged"]["values"])
                self.assertFalse(np.any(imaged == 0), name)
                for drone in method["drones"]:
                    self.assertEqual(len(drone["heading"]), len(drone["path"]))
                    self.assertTrue(all(a > 0 for a in drone["altitude"]))


if __name__ == "__main__":
    unittest.main()
