"""Hardware-free tests for stand_pose.py (run: cd ~/so101_work && python3 -m pytest test_stand_pose.py)."""
import json
from pathlib import Path

import pytest

import stand_pose as sp

CALIB = {k: v for k, v in json.loads(
    (Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_follower_arm.json").read_text()).items()}
REST = {j: float(v) for j, v in sp.REST_REF.items()}
STAND = {j: float(v) for j, v in sp.STAND_DEFAULT.items()}
STEP = sp.deg_s_to_ticks_per_s(5.0) / sp.RATE_HZ


def test_interpolation_matches_expected_waypoints():
    for p, expected in sp.EXPECTED_MID.items():
        mid = sp.interpolate(REST, STAND, p)
        # exact 50% elbow is 2048.5 (a rounding tie), so allow 1 tick
        assert all(abs(mid[j] - expected[j]) <= 1 for j in sp.MOVING)
        assert sum(mid[j] != expected[j] for j in sp.MOVING) <= 1


def test_interpolation_endpoints_and_held_joints():
    assert sp.interpolate(REST, STAND, 0.0) == {j: round(REST[j]) for j in sp.JOINTS}
    assert sp.interpolate(REST, STAND, 1.0) == {j: round(STAND[j]) for j in sp.JOINTS}


def test_largest_mover_gets_full_step_and_others_are_proportional():
    steps = sp.make_steps(REST, STAND, STEP)
    assert steps["elbow_flex"] == pytest.approx(STEP)
    assert steps["shoulder_lift"] == pytest.approx(STEP * 1135 / 2047)
    assert steps["wrist_flex"] == pytest.approx(STEP * 868 / 2047)


def test_all_joints_arrive_together_and_never_overshoot():
    ticks, arrived = sp.simulate(REST, STAND, STEP)
    assert ticks == pytest.approx(2047 / STEP, abs=1)
    assert max(arrived.values()) - min(arrived[j] for j in sp.MOVING) <= 1
    goal, steps = dict(REST), sp.make_steps(REST, STAND, STEP)
    for _ in range(ticks + 5):
        goal = sp.advance(goal, STAND, steps)
        for j in sp.JOINTS:
            lo, hi = sorted((REST[j], STAND[j]))
            assert lo <= goal[j] <= hi


def test_held_joints_do_not_move():
    start = dict(REST)
    stand = {j: (STAND[j] if j in sp.MOVING else start[j]) for j in sp.JOINTS}
    goal, steps = dict(start), sp.make_steps(start, stand, STEP)
    for _ in range(200):
        goal = sp.advance(goal, stand, steps)
    for j in ("shoulder_pan", "wrist_roll", "gripper"):
        assert goal[j] == start[j]


def test_zero_move_is_finished_immediately():
    ticks, _ = sp.simulate(REST, REST, STEP)
    assert ticks == 1


def test_duration_scales_with_speed():
    slow = sp.build_plan(REST, STAND, 5.0, 10.0, CALIB, 80)
    fast = sp.build_plan(REST, STAND, 10.0, 10.0, CALIB, 80)
    assert slow["move_s"] == pytest.approx(2 * fast["move_s"])
    assert slow["move_s"] == pytest.approx(2047 / (5.0 / 360 * 4096), rel=1e-6)
    assert slow["total_s"] == pytest.approx(2 * slow["move_s"] + 10.0)


def test_default_stand_passes_margin_check():
    assert sp.check_margin(STAND, CALIB, 80, sp.MOVING) == []


def test_stand_target_inside_margin_is_rejected_and_named():
    bad = dict(STAND, elbow_flex=CALIB["elbow_flex"]["range_min"] + 50)
    problems = sp.check_margin(bad, CALIB, 80, sp.MOVING)
    assert len(problems) == 1 and problems[0].startswith("elbow_flex")
    assert sp.check_margin(bad, CALIB, 40, sp.MOVING) == []  # fine with a smaller margin


def test_margin_boundary_is_inclusive():
    edge = dict(STAND, wrist_flex=CALIB["wrist_flex"]["range_max"] - 80)
    assert sp.check_margin(edge, CALIB, 80, sp.MOVING) == []
    over = dict(STAND, wrist_flex=CALIB["wrist_flex"]["range_max"] - 79)
    assert sp.check_margin(over, CALIB, 80, sp.MOVING) != []


def test_rest_pose_is_inside_range_but_violates_the_margin():
    # shoulder_lift rest 956 is 13 ticks above range_min 943: legal for the return, illegal under the stand margin
    assert sp.check_range(REST, CALIB, sp.JOINTS) == []
    assert sp.check_margin(REST, CALIB, 80, sp.MOVING) != []


def test_range_check_rejects_outside_range():
    bad = dict(REST, shoulder_lift=CALIB["shoulder_lift"]["range_min"] - 1)
    problems = sp.check_range(bad, CALIB, sp.JOINTS)
    assert len(problems) == 1 and problems[0].startswith("shoulder_lift")


def test_rest_deviation_tolerance():
    assert sp.check_rest_deviation(REST) == []
    near = dict(REST, elbow_flex=REST["elbow_flex"] - 150)
    assert sp.check_rest_deviation(near) == []
    far = dict(REST, elbow_flex=REST["elbow_flex"] - 151)
    assert [d[0] for d in sp.check_rest_deviation(far)] == ["elbow_flex"]
    raised = dict(STAND)
    assert {d[0] for d in sp.check_rest_deviation(raised)} == set(sp.MOVING)


def test_argument_limits(capsys):
    with pytest.raises(SystemExit):
        sp.parse_args(["--speed", "50.1"])
    with pytest.raises(SystemExit):
        sp.parse_args(["--speed", "0"])
    with pytest.raises(SystemExit):
        sp.parse_args(["--hold_s", "61"])
    with pytest.raises(SystemExit):
        sp.parse_args(["--max_lag", "0"])
    args = sp.parse_args([])
    assert (args.execute, args.speed, args.hold_s, args.margin, args.accel, args.max_lag) == (False, 5.0, 10.0, 80, 20, 150)
    assert "WARNING" not in capsys.readouterr().out


def test_speed_warning_only_above_15(capsys):
    sp.parse_args(["--speed", "15"])
    assert "WARNING" not in capsys.readouterr().out
    assert sp.parse_args(["--speed", "50"]).speed == 50.0
    out = capsys.readouterr().out
    assert "speed > 15 deg/s: higher inertia and servo load; keep hand on power" in out


def test_step_per_tick_at_higher_speeds():
    # largest mover (elbow, 2047 ticks) gets the full step; check the 50 Hz numbers quoted in the dry run
    for speed, expected in ((20, 4.551), (25, 5.689), (30, 6.827)):
        step = sp.deg_s_to_ticks_per_s(speed) / sp.RATE_HZ
        assert step == pytest.approx(expected, abs=1e-3)
        assert sp.make_steps(REST, STAND, step)["elbow_flex"] == pytest.approx(step)


def test_lag_detection_boundary_and_sign():
    goal, cur = dict(STAND), dict(STAND)
    assert sp.lagging_joints(goal, cur, 150) == []
    cur["elbow_flex"] = goal["elbow_flex"] - 150      # exactly max_lag: not lagging
    assert sp.lagging_joints(goal, cur, 150) == []
    cur["elbow_flex"] = goal["elbow_flex"] - 151      # goal is ahead of the servo
    assert sp.lagging_joints(goal, cur, 150) == [("elbow_flex", 151)]
    cur["elbow_flex"] = goal["elbow_flex"] + 151      # servo ahead of the goal also counts
    assert sp.lagging_joints(goal, cur, 150) == [("elbow_flex", -151)]


def test_lag_detection_reports_every_joint_and_honours_max_lag():
    goal = dict(STAND)
    cur = dict(STAND, shoulder_lift=STAND["shoulder_lift"] - 200, gripper=STAND["gripper"] + 400)
    assert [j for j, _ in sp.lagging_joints(goal, cur, 150)] == ["shoulder_lift", "gripper"]
    assert [j for j, _ in sp.lagging_joints(goal, cur, 300)] == ["gripper"]


def test_actuator_never_writes_in_dry_run():
    class Boom:
        def __getattr__(self, name):
            raise AssertionError(f"bus.{name} called during dry run")

    arm = sp.Actuator(Boom(), {j: i for i, j in enumerate(sp.JOINTS, 1)}, dry_run=True)
    arm.set_acceleration(20)
    arm.write_goals(REST)
    arm.torque(True)
    arm.torque(False)
    assert arm.torque_on is False
