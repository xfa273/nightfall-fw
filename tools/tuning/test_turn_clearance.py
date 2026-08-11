#!/usr/bin/env python3

from __future__ import annotations

import contextlib
import csv
import hashlib
import importlib.util
import io
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path, PurePosixPath


MODULE_PATH = Path(__file__).with_name("turn_clearance.py")
SPEC = importlib.util.spec_from_file_location("test_turn_clearance_module", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
TURN_CLEARANCE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = TURN_CLEARANCE
SPEC.loader.exec_module(TURN_CLEARANCE)


class MeasuredFootprintTest(unittest.TestCase):
    def test_default_footprint_matches_measured_machine_artifact(self):
        artifact_path = MODULE_PATH.parent / "data/mini_r2_0_footprint.json"
        artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
        footprint = TURN_CLEARANCE.Footprint()

        self.assertEqual(
            artifact["schema"], "nightfall_machine_footprint_v1"
        )
        self.assertTrue(
            artifact["reference_point"]["coincident_with_machine_centre"]
        )
        self.assertTrue(
            artifact["reference_point"]["coincident_with_turn_centre"]
        )
        self.assertEqual(
            artifact["extents_mm"],
            {
                "front": footprint.front_mm,
                "rear": footprint.rear_mm,
                "left": footprint.left_mm,
                "right": footprint.right_mm,
            },
        )
        self.assertAlmostEqual(
            artifact["overall_mm"]["length"],
            footprint.front_mm + footprint.rear_mm,
        )
        self.assertAlmostEqual(
            artifact["overall_mm"]["width"],
            footprint.left_mm + footprint.right_mm,
        )
        basis = TURN_CLEARANCE.footprint_basis(footprint)
        self.assertTrue(basis["uses_measured_default"])
        self.assertEqual(
            basis["measurement_artifact"],
            TURN_CLEARANCE.DEFAULT_FOOTPRINT_ARTIFACT,
        )


class PolygonDistanceTest(unittest.TestCase):
    def test_gap_touch_and_penetration_are_signed(self):
        square = ((0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0))
        separated = ((3.0, 0.0), (5.0, 0.0), (5.0, 2.0), (3.0, 2.0))
        touching = ((2.0, 0.0), (4.0, 0.0), (4.0, 2.0), (2.0, 2.0))
        overlapping = ((1.5, 0.0), (3.5, 0.0), (3.5, 2.0), (1.5, 2.0))
        self.assertAlmostEqual(
            TURN_CLEARANCE.polygon_signed_distance(square, separated).signed_distance_mm,
            1.0,
        )
        self.assertAlmostEqual(
            TURN_CLEARANCE.polygon_signed_distance(square, touching).signed_distance_mm,
            0.0,
        )
        self.assertLess(
            TURN_CLEARANCE.polygon_signed_distance(square, overlapping).signed_distance_mm,
            0.0,
        )

    def test_rotation_invariance(self):
        footprint = TURN_CLEARANCE.Footprint(2.0, 1.0, 1.0, 1.5)
        pose0 = TURN_CLEARANCE.PoseSample(0.0, "test", 0.0, 0.0, 0.0)
        pose90 = TURN_CLEARANCE.PoseSample(0.0, "test", 0.0, 0.0, 90.0)
        obstacle0 = ((4.0, -1.0), (5.0, -1.0), (5.0, 1.0), (4.0, 1.0))
        obstacle90 = ((-1.0, 4.0), (1.0, 4.0), (1.0, 5.0), (-1.0, 5.0))
        distance0 = TURN_CLEARANCE.polygon_signed_distance(
            footprint.polygon(pose0), obstacle0
        ).signed_distance_mm
        distance90 = TURN_CLEARANCE.polygon_signed_distance(
            footprint.polygon(pose90), obstacle90
        ).signed_distance_mm
        self.assertAlmostEqual(distance0, distance90, places=9)


class ClearanceEvaluationTest(unittest.TestCase):
    def test_standalone_post_is_checked_without_any_wall(self):
        footprint = TURN_CLEARANCE.Footprint(1.0, 1.0, 1.0, 1.0)
        obstacle = TURN_CLEARANCE.ObstacleRect("post", "post", -0.5, -0.5, 0.5, 0.5)
        scene = TURN_CLEARANCE.TurnScene(
            "post-only",
            0.0,
            0.0,
            0.0,
            TURN_CLEARANCE.TurnTarget(0.0, 0.0, 0.0),
            0.0,
            0.0,
            0.0,
            (obstacle,),
            "test",
        )
        result = TURN_CLEARANCE.evaluate_clearance(
            [TURN_CLEARANCE.PoseSample(0.0, "test", 0.0, 0.0, 0.0)],
            scene,
            footprint,
            TURN_CLEARANCE.ClearanceBudget(0.0, 0.0, 0.0, 0.0),
            0.1,
        )
        self.assertTrue(result.physical_collision)
        self.assertEqual(result.obstacle_kind, "post")

    def test_adaptive_interpolation_catches_between_frame_collision(self):
        footprint = TURN_CLEARANCE.Footprint(0.4, 0.4, 0.4, 0.4)
        obstacle = TURN_CLEARANCE.ObstacleRect("thin", "wall", 1.9, -0.2, 2.1, 0.2)
        scene = TURN_CLEARANCE.TurnScene(
            "between-frame",
            0.0,
            0.0,
            0.0,
            TURN_CLEARANCE.TurnTarget(0.0, 0.0, 0.0),
            0.0,
            0.0,
            0.0,
            (obstacle,),
            "test",
        )
        samples = [
            TURN_CLEARANCE.PoseSample(0.0, "test", 0.0, 0.0, 0.0),
            TURN_CLEARANCE.PoseSample(1.0, "test", 4.0, 0.0, 0.0),
        ]
        result = TURN_CLEARANCE.evaluate_clearance(
            samples,
            scene,
            footprint,
            TURN_CLEARANCE.ClearanceBudget(0.0, 0.0, 0.0, 0.0),
            0.1,
            max_corner_step_mm=0.1,
        )
        self.assertTrue(result.physical_collision)
        self.assertGreater(result.evaluated_pose_count, 2)

    def test_uncertainty_budget_is_separate_from_raw_gap(self):
        footprint = TURN_CLEARANCE.Footprint(1.0, 1.0, 1.0, 1.0)
        obstacle = TURN_CLEARANCE.ObstacleRect("wall", "wall", 3.0, -1.0, 4.0, 1.0)
        scene = TURN_CLEARANCE.TurnScene(
            "budget",
            0.0,
            0.0,
            0.0,
            TURN_CLEARANCE.TurnTarget(0.0, 0.0, 0.0),
            0.0,
            0.0,
            0.0,
            (obstacle,),
            "test",
        )
        result = TURN_CLEARANCE.evaluate_clearance(
            [TURN_CLEARANCE.PoseSample(0.0, "test", 0.0, 0.0, 0.0)],
            scene,
            footprint,
            TURN_CLEARANCE.ClearanceBudget(0.25, 0.25, 0.0, 0.0),
            0.75,
        )
        self.assertAlmostEqual(result.raw_min_clearance_mm, 2.0)
        self.assertAlmostEqual(result.effective_min_clearance_mm, 1.5)
        self.assertTrue(result.margin_passed)


class CanonicalSceneTest(unittest.TestCase):
    def test_absolute_video_requires_one_height_sidecar_per_trajectory(self):
        parser = TURN_CLEARANCE.build_arg_parser()
        args = parser.parse_args(
            [
                "video",
                "trajectory-1.csv",
                "trajectory-2.csv",
                "--mode",
                "2",
                "--code",
                "901",
                "--registration-mode",
                "absolute",
                "--scene-json",
                "scene.json",
            ]
        )
        with self.assertRaisesRegex(ValueError, "height-correction sidecar"):
            TURN_CLEARANCE._validate_args(args)

        args.height_correction_sidecar = [Path("one.calibration.json")]
        with self.assertRaisesRegex(ValueError, "once per trajectory"):
            TURN_CLEARANCE._validate_args(args)

    def test_absolute_video_cannot_weaken_geometry_or_completeness_gates(self):
        parser = TURN_CLEARANCE.build_arg_parser()
        base = [
            "video",
            "trajectory.csv",
            "--height-correction-sidecar",
            "trajectory.calibration.json",
            "--mode",
            "2",
            "--code",
            "901",
            "--registration-mode",
            "absolute",
            "--scene-json",
            "scene.json",
        ]
        cases = (
            ("--maximum-endpoint-error-mm", "5.1", "cannot exceed 5"),
            ("--maximum-heading-error-deg", "2.1", "cannot exceed 2"),
            ("--maximum-angle-shortfall-deg", "3.1", "cannot exceed 3"),
            ("--maximum-pose-gap-s", "0.051", "cannot exceed 0.050"),
            ("--max-corner-step-mm", "0.51", "cannot exceed 0.5"),
            ("--minimum-preroll-s", "0.039", "cannot be below 0.040"),
            ("--minimum-postroll-s", "0.019", "cannot be below 0.020"),
            ("--absolute-yaw-offset-deg", "-89", "require.*-90"),
        )
        for option, value, message in cases:
            with self.subTest(option=option):
                args = parser.parse_args([*base, option, value])
                with self.assertRaisesRegex(ValueError, message):
                    TURN_CLEARANCE._validate_args(args)

    def test_absolute_scene_binds_complete_topology_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            topology = root / "maze-topology.txt"
            topology.write_text("known maze revision\n", encoding="utf-8")
            raw = TURN_CLEARANCE.scene_to_dict(
                TURN_CLEARANCE.canonical_scene(901)
            )
            raw["bindings"] = {
                "board_layout_sha256": "12" * 32,
                "maze_topology_path": topology.name,
                "maze_topology_sha256": hashlib.sha256(
                    topology.read_bytes()
                ).hexdigest(),
            }
            raw["qualification"] = {
                "safety_qualified": True,
                "obstacle_inventory_complete": True,
            }
            scene_path = root / "scene.json"
            scene_path.write_text(json.dumps(raw), encoding="utf-8")
            result = TURN_CLEARANCE._absolute_scene_binding(scene_path)
            self.assertEqual(result["board_layout_sha256"], "12" * 32)
            self.assertEqual(
                result["maze_topology_path"], str(topology.resolve())
            )

            topology.write_text("tampered\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "does not match"):
                TURN_CLEARANCE._absolute_scene_binding(scene_path)

    def test_scene_report_round_trips_as_scene_input(self):
        original = TURN_CLEARANCE.canonical_scene(901)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scene.json"
            path.write_text(
                json.dumps(TURN_CLEARANCE.scene_to_dict(original)),
                encoding="utf-8",
            )
            loaded = TURN_CLEARANCE.load_scene(path)
        self.assertEqual(loaded, original)

    def test_scene_rejects_inconsistent_target_and_inverted_obstacle(self):
        raw = TURN_CLEARANCE.scene_to_dict(TURN_CLEARANCE.canonical_scene(901))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scene.json"
            raw["target_pose"]["x_mm"] += 1.0
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "inconsistent"):
                TURN_CLEARANCE.load_scene(path)
            raw = TURN_CLEARANCE.scene_to_dict(
                TURN_CLEARANCE.canonical_scene(901)
            )
            raw["obstacles"][0]["min_x_mm"] = raw["obstacles"][0]["max_x_mm"]
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "inverted"):
                TURN_CLEARANCE.load_scene(path)

    def test_cell_pitch_scales_target_start_and_obstacles_together(self):
        scene = TURN_CLEARANCE.canonical_scene(901, cell_pitch_mm=120.0)
        self.assertEqual(scene.start_y_mm, 60.0)
        self.assertEqual(scene.target.x_right_mm, 120.0)
        self.assertEqual(scene.target.y_forward_mm, 60.0)
        post_centres = {
            (
                0.5 * (item.min_x_mm + item.max_x_mm),
                0.5 * (item.min_y_mm + item.max_y_mm),
            )
            for item in scene.obstacles
            if item.kind == "post"
        }
        self.assertIn((60.0, 120.0), post_centres)

    def test_small90_exact_seed_clears_worst_legal_wall_union(self):
        results = []
        for code in (300, 400):
            args = TURN_CLEARANCE.build_arg_parser().parse_args(
                [
                    "evaluate",
                    "--code",
                    str(code),
                    "--velocity",
                    "300",
                    "--alpha",
                    "8920",
                    "--dist-in",
                    "5.956032993",
                    "--dist-out",
                    "5.956032993",
                ]
            )
            _, _, simulation = TURN_CLEARANCE.resolve_simulation(args)
            scene = TURN_CLEARANCE.canonical_scene(code)
            self.assertEqual(scene.start_y_mm, 0.0)
            self.assertEqual(scene.target.y_forward_mm, 45.0)
            result = TURN_CLEARANCE.evaluate_clearance(
                TURN_CLEARANCE.simulation_world_samples(simulation, scene),
                scene,
                TURN_CLEARANCE.Footprint(),
                TURN_CLEARANCE.ClearanceBudget(0.0, 0.0, 0.0, 0.0),
                0.0,
                max_corner_step_mm=0.1,
            )
            self.assertFalse(result.physical_collision)
            self.assertGreater(result.raw_min_clearance_mm, 10.0)
            self.assertLess(
                math.hypot(
                    simulation.final_pose.x_mm - scene.target.x_right_mm,
                    simulation.final_pose.y_mm - scene.target.y_forward_mm,
                ),
                0.25,
            )
            results.append(result)
        self.assertAlmostEqual(
            results[0].raw_min_clearance_mm,
            results[1].raw_min_clearance_mm,
            places=9,
        )


class D135RegressionTest(unittest.TestCase):
    def _args(self, *extra: str):
        parser = TURN_CLEARANCE.build_arg_parser()
        return parser.parse_args(
            ["evaluate", "--mode", "2", "--code", "901", *extra]
        )

    def _write_synthetic_trajectory(
        self,
        path: Path,
        simulation,
        *,
        cutoff_angle_deg: float | None = None,
        skipped_core_interval_ms: tuple[int, int] | None = None,
        append_postroll: bool = True,
    ) -> None:
        with path.open("w", newline="", encoding="ascii") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=(
                    "video_pts_s",
                    "x_mm",
                    "y_mm",
                    "yaw_deg_unwrapped",
                    "pose_valid",
                    "heading_valid",
                ),
            )
            writer.writeheader()
            for index in range(200):
                writer.writerow(
                    {
                        "video_pts_s": index / 1000.0,
                        "x_mm": 0.0,
                        "y_mm": 0.0,
                        "yaw_deg_unwrapped": 0.0,
                        "pose_valid": 1,
                        "heading_valid": 1,
                    }
                )
            last = None
            for sample in simulation.samples:
                if (
                    skipped_core_interval_ms is not None
                    and sample.phase == "turn_core"
                    and skipped_core_interval_ms[0]
                    <= sample.t_ms
                    < skipped_core_interval_ms[1]
                ):
                    continue
                writer.writerow(
                    {
                        "video_pts_s": 0.2 + sample.t_ms / 1000.0,
                        # Board yaw zero points along +x in the vision CSV.
                        "x_mm": sample.y_mm,
                        "y_mm": -sample.x_mm,
                        "yaw_deg_unwrapped": sample.theta_deg,
                        "pose_valid": 1,
                        "heading_valid": 1,
                    }
                )
                last = sample
                if (
                    cutoff_angle_deg is not None
                    and abs(sample.theta_deg) >= cutoff_angle_deg
                ):
                    append_postroll = False
                    break
            assert last is not None
            if append_postroll:
                for index in range(1, 51):
                    writer.writerow(
                        {
                            "video_pts_s": (
                                0.2 + last.t_ms / 1000.0 + index / 1000.0
                            ),
                            "x_mm": last.y_mm,
                            "y_mm": -last.x_mm,
                            "yaw_deg_unwrapped": last.theta_deg,
                            "pose_valid": 1,
                            "heading_valid": 1,
                        }
                    )

    def test_shortest_simulation_uses_runtime_accumulated_angle(self):
        args = self._args()
        _, turn, simulation = TURN_CLEARANCE.resolve_simulation(args)
        self.assertEqual(turn.signed_angle_deg, -135.0)
        self.assertAlmostEqual(simulation.final_pose.theta_deg, -135.0, places=3)

    def test_d135_empirical_sideslip_model_reproduces_video_core_endpoint(self):
        model_path = (
            MODULE_PATH.parent / "data/mode2_d135_in_empirical_model.json"
        )
        model = json.loads(model_path.read_text(encoding="utf-8"))
        coefficient = model["model"]["coefficient_s2_m"]
        args = self._args("--slip-angle-coefficient", str(coefficient))
        _, _, simulation = TURN_CLEARANCE.resolve_simulation(args)
        core = [
            sample for sample in simulation.samples if sample.phase == "turn_core"
        ][-1]
        observed = model["fit"]["observed_median_core_endpoint_mm"]
        residual = math.hypot(
            core.x_mm - observed["x_right_mm"],
            core.y_mm
            - simulation.turn.dist_in_mm
            - observed["y_forward_mm"],
        )
        self.assertAlmostEqual(
            residual, model["fit"]["fitted_endpoint_residual_mm"], places=9
        )
        self.assertLess(residual, 0.6)

    def test_robust_search_uses_worst_nominal_and_empirical_clearance(self):
        model_path = MODULE_PATH.parent / "data/mode2_d135_in_empirical_model.json"
        args = TURN_CLEARANCE.build_arg_parser().parse_args(
            [
                "search",
                "--mode",
                "2",
                "--code",
                "901",
                "--slip-angle-coefficient",
                "0.03303",
                "--slip-model-json",
                str(model_path),
                "--robust-slip-angle-coefficient",
                "0",
            ]
        )
        _, turn, _ = TURN_CLEARANCE.resolve_simulation(args)
        candidate = TURN_CLEARANCE._candidate_for_alpha(
            args,
            turn,
            TURN_CLEARANCE.canonical_scene(901),
            TURN_CLEARANCE.Footprint(),
            TURN_CLEARANCE.ClearanceBudget(),
            10250.0,
        )
        self.assertIsNotNone(candidate)
        assert candidate is not None
        result = candidate[0]
        self.assertEqual(result.clearance_model_slip_angle_coefficient, 0.0)
        self.assertFalse(result.model_scope_qualified)
        self.assertIn(
            "calibration artifact is not safety-qualified for swept-clearance recommendations",
            result.model_scope_violations,
        )
        self.assertFalse(result.clearance.margin_passed)

    def test_fixed_alpha_search_explores_endpoint_tolerance_offset_neighborhood(self):
        model_path = MODULE_PATH.parent / "data/mode2_d135_in_empirical_model.json"
        parser = TURN_CLEARANCE.build_arg_parser()
        with tempfile.TemporaryDirectory() as directory:
            report_path = Path(directory) / "fixed-alpha-search.json"
            args = parser.parse_args(
                [
                    "search",
                    "--mode",
                    "2",
                    "--code",
                    "901",
                    "--slip-angle-coefficient",
                    "0.03303",
                    "--slip-model-json",
                    str(model_path),
                    "--robust-slip-angle-coefficient",
                    "0",
                    "--alpha-min",
                    "10250",
                    "--alpha-max",
                    "10250",
                    "--alpha-step",
                    "250",
                    "--offset-quantum-mm",
                    "0.5",
                    "--report-json",
                    str(report_path),
                ]
            )
            with contextlib.redirect_stdout(io.StringIO()):
                return_code = TURN_CLEARANCE.command_search(args)
            report = json.loads(report_path.read_text(encoding="utf-8"))

        self.assertEqual(return_code, 1)
        self.assertEqual(report["search"]["evaluated_candidates"], 19)
        self.assertFalse(report["safe_recommendation_available"])
        self.assertIsNone(report["recommended"])
        diagnostic = report["best_diagnostic"]
        self.assertEqual(diagnostic["alpha_deg_s2"], 10250.0)
        self.assertEqual(diagnostic["dist_in_mm"], 24.5)
        self.assertEqual(diagnostic["dist_out_mm"], 33.0)
        self.assertLess(diagnostic["endpoint_error_mm"], 1.0)
        self.assertAlmostEqual(
            diagnostic["clearance"]["raw_min_clearance_mm"],
            4.781280561541932,
        )
        self.assertAlmostEqual(
            diagnostic["clearance"]["effective_min_clearance_mm"],
            0.6816424939947137,
        )
        self.assertFalse(diagnostic["clearance"]["margin_passed"])

    def test_zero_slip_search_is_diagnostic_until_swept_path_is_validated(self):
        parser = TURN_CLEARANCE.build_arg_parser()
        with tempfile.TemporaryDirectory() as directory:
            report_path = Path(directory) / "zero-slip-search.json"
            args = parser.parse_args(
                [
                    "search",
                    "--mode",
                    "2",
                    "--code",
                    "901",
                    "--alpha-min",
                    "10250",
                    "--alpha-max",
                    "10250",
                    "--offset-quantum-mm",
                    "0.5",
                    "--report-json",
                    str(report_path),
                ]
            )
            with contextlib.redirect_stdout(io.StringIO()):
                return_code = TURN_CLEARANCE.command_search(args)
            report = json.loads(report_path.read_text(encoding="utf-8"))

        self.assertEqual(return_code, 1)
        self.assertGreater(report["search"]["margin_passing_candidates"], 0)
        self.assertEqual(report["search"]["safety_qualified_candidates"], 0)
        self.assertFalse(report["safe_recommendation_available"])
        self.assertIsNone(report["recommended"])
        self.assertIn(
            "nominal zero-slip trajectory has no swept-path validation artifact",
            report["best_diagnostic"]["model_scope_violations"],
        )

    def test_offset_neighborhood_contains_every_endpoint_valid_grid_pair(self):
        args = TURN_CLEARANCE.build_arg_parser().parse_args(
            [
                "search",
                "--mode",
                "2",
                "--code",
                "901",
                "--alpha-min",
                "10250",
                "--alpha-max",
                "10250",
                "--minimum-offset-mm",
                "20",
                "--maximum-offset-mm",
                "35",
                "--offset-quantum-mm",
                "0.5",
            ]
        )
        scene = TURN_CLEARANCE.canonical_scene(901)
        _, _, core = TURN_CLEARANCE.resolve_simulation(
            args, {"alpha": 10250.0, "dist_in": 0.0, "dist_out": 0.0}
        )
        enumerated = set(TURN_CLEARANCE._offset_pairs_for_alpha(args, scene, core))
        valid = set()
        for in_index in range(40, 71):
            for out_index in range(40, 71):
                dist_in = in_index * 0.5
                dist_out = out_index * 0.5
                _, _, simulation = TURN_CLEARANCE.resolve_simulation(
                    args,
                    {
                        "alpha": 10250.0,
                        "dist_in": dist_in,
                        "dist_out": dist_out,
                    },
                )
                endpoint_error = math.hypot(
                    simulation.final_pose.x_mm - scene.target.x_right_mm,
                    simulation.final_pose.y_mm - scene.target.y_forward_mm,
                )
                heading_error = abs(
                    simulation.final_pose.theta_deg - scene.target.theta_deg
                )
                if (
                    endpoint_error <= args.maximum_endpoint_error_mm
                    and heading_error <= args.maximum_heading_error_deg
                ):
                    valid.add((dist_in, dist_out))
        self.assertTrue(valid)
        self.assertTrue(valid.issubset(enumerated))

    def test_180_offset_neighborhood_contains_singular_grid_pairs(self):
        args = TURN_CLEARANCE.build_arg_parser().parse_args(
            [
                "search",
                "--mode",
                "2",
                "--code",
                "502",
                "--alpha-min",
                "4400",
                "--alpha-max",
                "4400",
                "--minimum-offset-mm",
                "0",
                "--maximum-offset-mm",
                "10",
                "--offset-quantum-mm",
                "1",
            ]
        )
        scene = TURN_CLEARANCE.canonical_scene(502)
        _, _, core = TURN_CLEARANCE.resolve_simulation(
            args, {"alpha": 4400.0, "dist_in": 0.0, "dist_out": 0.0}
        )
        enumerated = set(TURN_CLEARANCE._offset_pairs_for_alpha(args, scene, core))
        valid = set()
        for dist_in in range(11):
            for dist_out in range(11):
                _, _, simulation = TURN_CLEARANCE.resolve_simulation(
                    args,
                    {
                        "alpha": 4400.0,
                        "dist_in": float(dist_in),
                        "dist_out": float(dist_out),
                    },
                )
                endpoint_error = math.hypot(
                    simulation.final_pose.x_mm - scene.target.x_right_mm,
                    simulation.final_pose.y_mm - scene.target.y_forward_mm,
                )
                heading_error = abs(
                    simulation.final_pose.theta_deg - scene.target.theta_deg
                )
                if (
                    endpoint_error <= args.maximum_endpoint_error_mm
                    and heading_error <= args.maximum_heading_error_deg
                ):
                    valid.add((float(dist_in), float(dist_out)))
        self.assertTrue(valid)
        self.assertTrue(valid.issubset(enumerated))

    def test_empirical_model_scope_blocks_unbound_and_extrapolated_search(self):
        parser = TURN_CLEARANCE.build_arg_parser()
        model_path = MODULE_PATH.parent / "data/mode2_d135_in_empirical_model.json"
        common = [
            "search",
            "--mode",
            "2",
            "--code",
            "901",
            "--slip-angle-coefficient",
            "0.03303",
        ]
        unbound = parser.parse_args(common)
        _, unbound_turn, _ = TURN_CLEARANCE.resolve_simulation(unbound)
        unbound_candidate = TURN_CLEARANCE._candidate_for_alpha(
            unbound,
            unbound_turn,
            TURN_CLEARANCE.canonical_scene(901),
            TURN_CLEARANCE.Footprint(),
            TURN_CLEARANCE.ClearanceBudget(),
            10250.0,
        )
        self.assertIsNotNone(unbound_candidate)
        assert unbound_candidate is not None
        self.assertFalse(unbound_candidate[0].model_scope_qualified)

        bound = parser.parse_args(
            [*common, "--slip-model-json", str(model_path)]
        )
        _, bound_turn, _ = TURN_CLEARANCE.resolve_simulation(bound)
        in_scope = TURN_CLEARANCE._candidate_for_alpha(
            bound,
            bound_turn,
            TURN_CLEARANCE.canonical_scene(901),
            TURN_CLEARANCE.Footprint(),
            TURN_CLEARANCE.ClearanceBudget(),
            10250.0,
        )
        out_of_scope = TURN_CLEARANCE._candidate_for_alpha(
            bound,
            bound_turn,
            TURN_CLEARANCE.canonical_scene(901),
            TURN_CLEARANCE.Footprint(),
            TURN_CLEARANCE.ClearanceBudget(),
            11000.0,
        )
        self.assertIsNotNone(in_scope)
        self.assertIsNotNone(out_of_scope)
        assert in_scope is not None and out_of_scope is not None
        self.assertFalse(in_scope[0].model_scope_qualified)
        self.assertIn(
            "calibration artifact is not safety-qualified for swept-clearance recommendations",
            in_scope[0].model_scope_violations,
        )
        self.assertFalse(out_of_scope[0].model_scope_qualified)
        self.assertIn(
            "angular acceleration is outside calibration",
            out_of_scope[0].model_scope_violations,
        )

        with tempfile.TemporaryDirectory() as directory:
            qualified_model_path = Path(directory) / "qualified-model.json"
            qualified_model = json.loads(model_path.read_text(encoding="utf-8"))
            qualified_model["qualification"]["safety_qualified"] = True
            qualified_model_path.write_text(
                json.dumps(qualified_model), encoding="utf-8"
            )
            qualified = parser.parse_args(
                [*common, "--slip-model-json", str(qualified_model_path)]
            )
            _, qualified_turn, _ = TURN_CLEARANCE.resolve_simulation(qualified)
            qualified_in_scope = TURN_CLEARANCE._candidate_for_alpha(
                qualified,
                qualified_turn,
                TURN_CLEARANCE.canonical_scene(901),
                TURN_CLEARANCE.Footprint(),
                TURN_CLEARANCE.ClearanceBudget(),
                10250.0,
            )
            qualified_out_of_scope = TURN_CLEARANCE._candidate_for_alpha(
                qualified,
                qualified_turn,
                TURN_CLEARANCE.canonical_scene(901),
                TURN_CLEARANCE.Footprint(),
                TURN_CLEARANCE.ClearanceBudget(),
                11000.0,
            )
            self.assertIsNotNone(qualified_in_scope)
            self.assertIsNotNone(qualified_out_of_scope)
            assert qualified_in_scope is not None
            assert qualified_out_of_scope is not None
            self.assertTrue(qualified_in_scope[0].model_scope_qualified)
            self.assertFalse(qualified_out_of_scope[0].model_scope_qualified)

            qualified_nominal_path = Path(directory) / "qualified-nominal.json"
            qualified_nominal = json.loads(
                qualified_model_path.read_text(encoding="utf-8")
            )
            qualified_nominal["model"]["coefficient_s2_m"] = 0.0
            qualified_nominal_path.write_text(
                json.dumps(qualified_nominal), encoding="utf-8"
            )
            nominal_args = parser.parse_args(
                [
                    "search",
                    "--mode",
                    "2",
                    "--code",
                    "901",
                    "--slip-model-json",
                    str(qualified_nominal_path),
                ]
            )
            _, nominal_turn, _ = TURN_CLEARANCE.resolve_simulation(nominal_args)
            nominal_candidate = TURN_CLEARANCE._candidate_for_alpha(
                nominal_args,
                nominal_turn,
                TURN_CLEARANCE.canonical_scene(901),
                TURN_CLEARANCE.Footprint(),
                TURN_CLEARANCE.ClearanceBudget(),
                10250.0,
            )
            self.assertIsNotNone(nominal_candidate)
            assert nominal_candidate is not None
            self.assertTrue(nominal_candidate[0].model_scope_qualified)

            report_path = Path(directory) / "scope-search.json"
            command_args = parser.parse_args(
                [
                    *common,
                    "--slip-model-json",
                    str(model_path),
                    "--alpha-min",
                    "11000",
                    "--alpha-max",
                    "11000",
                    "--report-json",
                    str(report_path),
                ]
            )
            with contextlib.redirect_stdout(io.StringIO()):
                return_code = TURN_CLEARANCE.command_search(command_args)
            report = json.loads(report_path.read_text(encoding="utf-8"))
        self.assertEqual(return_code, 1)
        self.assertFalse(report["safe_recommendation_available"])
        self.assertIsNone(report["recommended"])
        self.assertFalse(report["best_diagnostic"]["model_scope_qualified"])

        with tempfile.TemporaryDirectory() as directory:
            report_path = Path(directory) / "diagnostic-only-search.json"
            command_args = parser.parse_args(
                [
                    *common,
                    "--slip-model-json",
                    str(model_path),
                    "--alpha-min",
                    "10250",
                    "--alpha-max",
                    "10250",
                    "--offset-quantum-mm",
                    "0.5",
                    "--report-json",
                    str(report_path),
                ]
            )
            with contextlib.redirect_stdout(io.StringIO()):
                return_code = TURN_CLEARANCE.command_search(command_args)
            report = json.loads(report_path.read_text(encoding="utf-8"))
        self.assertEqual(return_code, 1)
        self.assertGreater(report["search"]["margin_passing_candidates"], 0)
        self.assertEqual(report["search"]["safety_qualified_candidates"], 0)
        self.assertFalse(report["safe_recommendation_available"])
        self.assertIsNone(report["recommended"])

    def test_current_d135_hits_inner_post_but_canonical_closure_clears_it(self):
        args = self._args()
        _, _, current = TURN_CLEARANCE.resolve_simulation(args)
        scene = TURN_CLEARANCE.canonical_scene(901)
        footprint = TURN_CLEARANCE.Footprint()
        zero_budget = TURN_CLEARANCE.ClearanceBudget(0.0, 0.0, 0.0, 0.0)
        current_result = TURN_CLEARANCE.evaluate_clearance(
            TURN_CLEARANCE.simulation_world_samples(current, scene),
            scene,
            footprint,
            zero_budget,
            0.0,
        )
        self.assertTrue(current_result.physical_collision)
        self.assertIn("post", current_result.obstacle_id)

        _, _, candidate = TURN_CLEARANCE.resolve_simulation(
            args,
            {"alpha": 10250.0, "dist_in": 29.0, "dist_out": 21.5},
        )
        candidate_result = TURN_CLEARANCE.evaluate_clearance(
            TURN_CLEARANCE.simulation_world_samples(candidate, scene),
            scene,
            footprint,
            zero_budget,
            0.0,
        )
        self.assertFalse(candidate_result.physical_collision)
        self.assertGreater(candidate_result.raw_min_clearance_mm, 7.0)
        self.assertLess(
            math.hypot(
                candidate.final_pose.x_mm - scene.target.x_right_mm,
                candidate.final_pose.y_mm - scene.target.y_forward_mm,
            ),
            0.25,
        )
        strict_report = TURN_CLEARANCE._sim_report(
            candidate,
            scene,
            footprint,
            zero_budget,
            candidate_result,
            0.1,
            0.1,
        )
        self.assertTrue(strict_report["acceptance"]["clearance_passed"])
        self.assertFalse(strict_report["acceptance"]["endpoint_passed"])
        self.assertFalse(strict_report["acceptance"]["passed"])

    def test_aabb_pruning_matches_full_obstacle_scan(self):
        args = self._args()
        scene = TURN_CLEARANCE.canonical_scene(901)
        footprint = TURN_CLEARANCE.Footprint()
        zero_budget = TURN_CLEARANCE.ClearanceBudget(0.0, 0.0, 0.0, 0.0)
        simulations = [
            TURN_CLEARANCE.resolve_simulation(args)[2],
            TURN_CLEARANCE.resolve_simulation(
                args,
                {"alpha": 10250.0, "dist_in": 29.0, "dist_out": 21.5},
            )[2],
        ]
        for simulation in simulations:
            samples = TURN_CLEARANCE.simulation_world_samples(simulation, scene)
            optimized = TURN_CLEARANCE.evaluate_clearance(
                samples,
                scene,
                footprint,
                zero_budget,
                0.0,
                0.5,
            )
            dense = TURN_CLEARANCE.densify_poses(samples, footprint, 0.5)
            brute_force = min(
                TURN_CLEARANCE.polygon_signed_distance(
                    footprint.polygon(pose), obstacle.polygon()
                ).signed_distance_mm
                for pose in dense
                for obstacle in scene.obstacles
            )
            self.assertAlmostEqual(
                optimized.raw_min_clearance_mm, brute_force, places=9
            )

    def test_left_and_right_canonical_scenes_are_mirrored(self):
        results = []
        for code in (901, 902):
            args = self._args("--code", str(code))
            _, _, simulation = TURN_CLEARANCE.resolve_simulation(args)
            scene = TURN_CLEARANCE.canonical_scene(code)
            result = TURN_CLEARANCE.evaluate_clearance(
                TURN_CLEARANCE.simulation_world_samples(simulation, scene),
                scene,
                TURN_CLEARANCE.Footprint(),
                TURN_CLEARANCE.ClearanceBudget(0.0, 0.0, 0.0, 0.0),
                0.0,
            )
            results.append(result)
        self.assertAlmostEqual(
            results[0].raw_min_clearance_mm,
            results[1].raw_min_clearance_mm,
            places=9,
        )
        self.assertAlmostEqual(
            results[0].first_collision_theta_deg,
            -results[1].first_collision_theta_deg,
            places=9,
        )

    def test_video_extractor_recovers_a_synthetic_firmware_turn(self):
        args = self._args()
        _, turn, simulation = TURN_CLEARANCE.resolve_simulation(args)
        scene = TURN_CLEARANCE.canonical_scene(901)
        footprint = TURN_CLEARANCE.Footprint()
        budget = TURN_CLEARANCE.ClearanceBudget(0.0, 0.0, 0.0, 0.0)
        nominal = TURN_CLEARANCE.evaluate_clearance(
            TURN_CLEARANCE.simulation_world_samples(simulation, scene),
            scene,
            footprint,
            budget,
            0.0,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectory.csv"
            self._write_synthetic_trajectory(path, simulation)
            measured_samples, metadata = TURN_CLEARANCE.extract_video_turn(
                path,
                scene,
                turn,
                simulation,
                turn_index=1,
                omega_threshold_deg_s=80.0,
            )
            self.assertEqual(
                metadata["sha256"], hashlib.sha256(path.read_bytes()).hexdigest()
            )
        measured = TURN_CLEARANCE.evaluate_clearance(
            measured_samples,
            scene,
            footprint,
            budget,
            0.0,
        )
        self.assertEqual(metadata["active_region_count"], 1)
        self.assertGreater(metadata["angle_progress_deg"], 133.0)
        self.assertAlmostEqual(
            metadata["estimated_primitive_origin_board_x_mm"], 0.0, delta=0.75
        )
        self.assertAlmostEqual(
            measured.raw_min_clearance_mm,
            nominal.raw_min_clearance_mm,
            delta=1.0,
        )

    def test_video_extractor_rejects_truncated_turns(self):
        args = self._args()
        _, turn, simulation = TURN_CLEARANCE.resolve_simulation(args)
        scene = TURN_CLEARANCE.canonical_scene(901)
        with tempfile.TemporaryDirectory() as directory:
            for cutoff in (85.0, 100.0, 120.0):
                path = Path(directory) / f"trajectory-{cutoff:.0f}.csv"
                self._write_synthetic_trajectory(
                    path, simulation, cutoff_angle_deg=cutoff
                )
                with self.assertRaises(ValueError):
                    TURN_CLEARANCE.extract_video_turn(
                        path, scene, turn, simulation
                    )

    def test_video_extractor_rejects_large_tracking_gap(self):
        args = self._args()
        _, turn, simulation = TURN_CLEARANCE.resolve_simulation(args)
        scene = TURN_CLEARANCE.canonical_scene(901)
        core = [sample for sample in simulation.samples if sample.phase == "turn_core"]
        interval = (core[75].t_ms, core[130].t_ms)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectory-gap.csv"
            self._write_synthetic_trajectory(
                path, simulation, skipped_core_interval_ms=interval
            )
            with self.assertRaisesRegex(ValueError, "valid-pose gap"):
                TURN_CLEARANCE.extract_video_turn(path, scene, turn, simulation)

    def test_absolute_video_registration_preserves_trial_translation(self):
        args = self._args("--dist-in", "29", "--dist-out", "21.5")
        _, turn, simulation = TURN_CLEARANCE.resolve_simulation(args)
        scene = TURN_CLEARANCE.canonical_scene(901)
        world_samples = TURN_CLEARANCE.simulation_world_samples(simulation, scene)
        results = []
        with tempfile.TemporaryDirectory() as directory:
            for shift_x_mm in (0.0, 10.0):
                path = Path(directory) / f"absolute-{shift_x_mm:.0f}.csv"
                with path.open("w", newline="", encoding="ascii") as stream:
                    writer = csv.DictWriter(
                        stream,
                        fieldnames=(
                            "video_pts_s",
                            "x_mm",
                            "y_mm",
                            "yaw_deg_unwrapped",
                            "pose_valid",
                            "heading_valid",
                        ),
                    )
                    writer.writeheader()
                    for index in range(200):
                        writer.writerow(
                            {
                                "video_pts_s": index / 1000.0,
                                "x_mm": scene.start_x_mm + shift_x_mm,
                                "y_mm": scene.start_y_mm,
                                # Vision yaw zero points along +board-x; scene
                                # theta zero points along +world-y.
                                "yaw_deg_unwrapped": scene.start_heading_deg + 90.0,
                                "pose_valid": 1,
                                "heading_valid": 1,
                            }
                        )
                    for sample in world_samples[1:]:
                        writer.writerow(
                            {
                                "video_pts_s": 0.2 + sample.time_s,
                                "x_mm": sample.x_mm + shift_x_mm,
                                "y_mm": sample.y_mm,
                                "yaw_deg_unwrapped": sample.theta_deg + 90.0,
                                "pose_valid": 1,
                                "heading_valid": 1,
                            }
                        )
                    final = world_samples[-1]
                    for index in range(1, 51):
                        writer.writerow(
                            {
                                "video_pts_s": 0.2 + final.time_s + index / 1000.0,
                                "x_mm": final.x_mm + shift_x_mm,
                                "y_mm": final.y_mm,
                                "yaw_deg_unwrapped": final.theta_deg + 90.0,
                                "pose_valid": 1,
                                "heading_valid": 1,
                            }
                        )
                samples, metadata = TURN_CLEARANCE.extract_video_turn(
                    path,
                    scene,
                    turn,
                    simulation,
                    registration_mode="absolute",
                )
                self.assertEqual(metadata["registration_mode"], "absolute")
                results.append(
                    TURN_CLEARANCE.evaluate_clearance(
                        samples,
                        scene,
                        TURN_CLEARANCE.Footprint(),
                        TURN_CLEARANCE.ClearanceBudget(0.0, 0.0, 0.0, 0.0),
                        0.0,
                    )
                )
        self.assertGreater(results[0].raw_min_clearance_mm, 7.0)
        self.assertLess(results[1].raw_min_clearance_mm, 2.0)


class CalibrationManifestTest(unittest.TestCase):
    @staticmethod
    def _sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            while chunk := stream.read(1024 * 1024):
                digest.update(chunk)
        return digest.hexdigest()

    def test_d135_manifest_has_portable_parameter_and_source_digests(self):
        path = MODULE_PATH.parent / "data/mode2_d135_in_manifest.json"
        manifest = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(manifest["schema"], "nightfall_turn_calibration_manifest_v1")
        self.assertEqual(
            manifest["default_clearance_footprint"],
            TURN_CLEARANCE.DEFAULT_FOOTPRINT_ARTIFACT,
        )
        self.assertEqual(len(manifest["datasets"]), 10)
        for dataset in manifest["datasets"]:
            params = dataset["params"]
            values = [
                params[name]
                for name in (
                    "velocity_mm_s",
                    "alpha_deg_s2",
                    "configured_angle_deg",
                    "effective_angle_deg",
                    "dist_in_mm",
                    "dist_out_mm",
                )
            ]
            record = "|".join(format(float(value), ".9g") for value in values)
            self.assertEqual(
                hashlib.sha256(record.encode("ascii")).hexdigest(),
                dataset["params_record_sha256"],
            )
            for digest_key in ("report_sha256", "trajectory_set_sha256"):
                if digest_key in dataset:
                    self.assertEqual(len(dataset[digest_key]), 64)
                    int(dataset[digest_key], 16)
            for key in ("report", "video", "hfr_report", "trajectory", "trace"):
                if key in dataset:
                    relative = PurePosixPath(dataset[key])
                    self.assertFalse(relative.is_absolute())
                    self.assertNotIn("..", relative.parts)
                    digest_key = key + "_sha256"
                    self.assertIn(digest_key, dataset)
                    self.assertEqual(len(dataset[digest_key]), 64)
                    int(dataset[digest_key], 16)
                    source = MODULE_PATH.parents[2] / relative
                    # Raw sessions stay ignored.  Clean-checkout CI validates
                    # schema/digests; a populated local checkout also detects
                    # accidental mutation of every directly catalogued file.
                    if source.is_file():
                        self.assertEqual(self._sha256(source), dataset[digest_key])
        corpus_records = sorted(
            (
                f"{dataset['id']}  {dataset['report_sha256']}  "
                f"{dataset['trajectory_set_sha256']}\n"
            )
            for dataset in manifest["datasets"]
            if "report_sha256" in dataset
        )
        corpus_digest = hashlib.sha256(
            "".join(corpus_records).encode("utf-8")
        ).hexdigest()
        self.assertEqual(
            corpus_digest,
            "0955902ea80dbd936401ff197d8de944897d6f89b7c53249818ac401991168cf",
        )
        self.assertEqual(
            corpus_digest, manifest["non_contact_corpus_sha256"]
        )
        model = json.loads(
            (MODULE_PATH.parent / "data/mode2_d135_in_empirical_model.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(
            model["schema"], "nightfall_turn_empirical_sideslip_model_v1"
        )
        self.assertEqual(
            model["default_clearance_footprint"],
            TURN_CLEARANCE.DEFAULT_FOOTPRINT_ARTIFACT,
        )
        self.assertEqual(model["turn"]["mode"], 2)
        self.assertEqual(model["turn"]["code"], 901)
        self.assertEqual(model["turn"]["side"], "right")
        self.assertEqual(model["turn"]["velocity_mm_s"], 500.0)
        self.assertEqual(model["turn"]["alpha_deg_s2"], 10250.0)
        self.assertEqual(model["turn"]["effective_angle_deg"], 135.0)
        self.assertTrue(model["turn"]["positive_entry_regime_only"])
        self.assertEqual(model["model"]["coefficient_s2_m"], 0.03303)
        self.assertFalse(model["qualification"]["safety_qualified"])
        self.assertEqual(
            model["fit"]["audit"][
                "trajectory_start_corrected_endpoint_fit_coefficient_s2_m"
            ],
            0.0261,
        )
        self.assertEqual(
            model["fit"]["audit"]["full_path_fit_coefficient_s2_m"],
            0.0178,
        )
        self.assertEqual(
            model["fit"]["audit"]["full_path_fit_rmse_mm"], 2.89
        )
        audit = model["fit"]["audit"]
        self.assertEqual(
            audit["corpus_manifest"],
            "tools/tuning/data/mode2_d135_in_manifest.json",
        )
        self.assertEqual(
            audit["non_contact_corpus_sha256"],
            manifest["non_contact_corpus_sha256"],
        )
        self.assertEqual(audit["dataset_count"], 9)
        self.assertEqual(audit["trajectory_count"], 27)
        self.assertEqual(audit["full_path_heading_range_deg"], [10.0, 130.0])
        source = model["source"]
        self.assertEqual(
            source["manifest"],
            "tools/tuning/data/mode2_d135_in_manifest.json",
        )
        dataset = next(
            item
            for item in manifest["datasets"]
            if item["id"] == source["dataset_id"]
        )
        for key in (
            "params_record_sha256",
            "report_sha256",
            "trajectory_set_sha256",
        ):
            self.assertEqual(source[key], dataset[key])
        self.assertEqual(
            model["turn"]["velocity_mm_s"], dataset["params"]["velocity_mm_s"]
        )
        self.assertEqual(
            model["turn"]["alpha_deg_s2"], dataset["params"]["alpha_deg_s2"]
        )
        self.assertEqual(
            model["turn"]["effective_angle_deg"],
            dataset["params"]["effective_angle_deg"],
        )


if __name__ == "__main__":
    unittest.main()
