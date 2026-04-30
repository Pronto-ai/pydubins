#!/usr/bin/python
try:
    import unittest2 as unittest
except:
    import unittest
import math
import dubins

class DubinsCurvesTests(unittest.TestCase):

    def test_colocated_configurations(self):
        # Regression: previous version of the code did not work correctly 
        # for goal and initial configurations were too close together
        qi = (0,0,0)
        qg = (0,0,0)
        turning_radius = 1.0
        # not raising an exception is sufficient
        dubins.shortest_path(qi, qg, turning_radius)

    def test_inclusion_of_path_sample(self):
        pts, dists = dubins.path_sample((0,0,0), (1,0,0), 1.0, 0.1)
        self.assertTrue(len(pts) >= 10)
        self.assertEqual(len(pts), len(dists))

    def test_invalid_turning_radius(self):
        with self.assertRaises(RuntimeError):
            dubins.shortest_path((0,0,0), (1,0,0), -1.0)

    def test_simple(self):
        d = dubins.shortest_path((0,0,0), (10,0,0), 1.0)
        length = d.path_length()
        self.assertAlmostEqual(length, 10.0)

    def test_almost_full_loop(self):
        r = 2.0
        dy = 0.1
        d = dubins.shortest_path((0,0,0), (0,-dy,0), r)
        length = d.path_length()
        self.assertAlmostEqual(length, 2*math.pi*r + dy)

    def test_easy_lrl(self):
        # Regression for https://github.com/AndrewWalker/Dubins-Curves/issues/2
        r = 1.0
        q0 = (0, 0, math.pi/2.)
        q1 = (1, 0, -math.pi/2.)
        d = dubins.shortest_path(q0, q1, r)
        path_type = d.path_type()
        self.assertAlmostEqual(path_type, dubins.LRL)

    def test_half_loop(self):
        r = 1.0
        dx = r * math.sqrt(2)/2
        dy = r * math.sqrt(2)/2
        d = dubins.shortest_path((0,0,0), (0, 2*r, -math.pi), r)
        length = d.path_length()
        self.assertAlmostEqual(length, math.pi*r)

    def test_non_unit_turning_radius(self):
        d = dubins.shortest_path((0,0,0), (10,0,0), 2.0)
        length = d.path_length()
        self.assertAlmostEqual(length, 10.0)

    def test_turning_radius_scaling(self):
        a = dubins.shortest_path((0,0,0), (10,10,math.pi/4.0), 1.0).path_length()
        b = dubins.shortest_path((0,0,0), (10,10,math.pi/4.0), 2.0).path_length()
        self.assert_(b > a)

    def test_readme_demo(self):
        q0 = (0, 0, math.pi/2)
        q1 = (1, 1, 0.0)
        turning_radius = 1.0
        step_size = 0.5

        path = dubins.shortest_path(q0, q1, turning_radius)
        configurations, _ = path.sample_many(step_size)

    def test_sample_many_callback_contract(self):
        # Regression test for the Cython-3 callback signature fix
        # (DubinsPathSamplingCallback `except? -1`). Previously the module
        # either failed to build on Cython>=3 or, if the callback was
        # mis-declared `noexcept`, exceptions raised inside the sampling
        # loop were silently swallowed. This test exercises the sampling
        # callback end-to-end and asserts the returned data is well-formed.
        q0 = (0.0, 0.0, 0.0)
        q1 = (10.0, 0.0, 0.0)
        turning_radius = 1.0
        step_size = 0.1

        path = dubins.shortest_path(q0, q1, turning_radius)
        configs, dists = path.sample_many(step_size)

        # callback must have been invoked many times
        self.assertGreater(len(configs), 50)
        # one distance per configuration
        self.assertEqual(len(configs), len(dists))
        # first sample must correspond to the start configuration
        self.assertAlmostEqual(dists[0], 0.0)
        for a, b in zip(q0, configs[0]):
            self.assertAlmostEqual(a, b)
        # distances must be monotonically increasing and bounded by the
        # total path length
        for prev, cur in zip(dists, dists[1:]):
            self.assertGreater(cur, prev)
        self.assertLessEqual(dists[-1], path.path_length())
        # and the last reported distance should be within one step of the end
        self.assertLess(path.path_length() - dists[-1], step_size)


