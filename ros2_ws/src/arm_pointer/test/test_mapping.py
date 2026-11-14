import math

from arm_pointer.mapping import in_limits, pan_to_raw, raw_to_rad, step_toward


def test_pan_front_is_front_raw():
    assert pan_to_raw(1.0, 0.0, 2048, 1) == 2048


def test_pan_left_90deg():
    assert math.isclose(pan_to_raw(0.0, 1.0, 2048, 1), 2048 + 1024)
    assert math.isclose(pan_to_raw(0.0, 1.0, 2048, -1), 2048 - 1024)


def test_raw_to_rad():
    assert raw_to_rad(2048) == 0.0
    assert math.isclose(raw_to_rad(3072), math.pi / 2)


def test_step_toward_never_overshoots():
    assert step_toward(0.0, 10.0, 3.0) == 3.0
    assert step_toward(9.0, 10.0, 3.0) == 10.0
    assert step_toward(0.0, -10.0, 3.0) == -3.0


def test_in_limits():
    assert in_limits(5, 0, 10) and not in_limits(11, 0, 10)


def test_synced_steps_arrive_together():
    from arm_pointer.mapping import synced_steps

    cur = {"a": 0.0, "b": 0.0, "c": 100.0}
    tgt = {"a": 100.0, "b": 25.0, "c": 100.0}
    steps = synced_steps(cur, tgt, 4.0)
    assert steps["a"] == 4.0
    assert math.isclose(steps["b"], 1.0)
    assert steps["c"] == 4.0  # already there, step is irrelevant but must stay positive
    assert math.isclose(abs(tgt["a"] - cur["a"]) / steps["a"], abs(tgt["b"] - cur["b"]) / steps["b"])


def test_synced_steps_nothing_to_move():
    from arm_pointer.mapping import synced_steps

    assert synced_steps({"a": 5.0}, {"a": 5.0}, 3.0) == {"a": 3.0}


def test_watchdog_state():
    from arm_pointer.mapping import watchdog_state

    assert watchdog_state(None, 1.0, 10.0) == "idle"
    assert watchdog_state(0.0, 1.0, 10.0) == "track"
    assert watchdog_state(0.99, 1.0, 10.0) == "track"
    assert watchdog_state(1.0, 1.0, 10.0) == "hold"
    assert watchdog_state(9.99, 1.0, 10.0) == "hold"
    assert watchdog_state(10.0, 1.0, 10.0) == "return"
    assert watchdog_state(50.0, 1.0, 10.0) == "return"


def _wrist_and_hand(phis):
    """Forward model: wrist position relative to the shoulder and the hand direction (deg) for joint angles phis."""
    from arm_pointer.mapping import segment_pitches, wrist_from_shoulder

    return wrist_from_shoulder(phis[0], phis[1]), segment_pitches(*phis)[2]


def test_planar_model_matches_urdf_pose_read_on_the_hardware():
    from arm_pointer.mapping import raw_to_phi, segment_pitches

    # pose read with the arm laid out by hand on 2026-11-02 (photo): upper arm slightly down, forearm level, hand up
    phis = [raw_to_phi(r, 2048) for r in (3110, 990, 1054)]
    tu, tf, th = segment_pitches(*phis)
    assert math.isclose(tu, -17.4, abs_tol=1.0) and math.isclose(tf, 1.6, abs_tol=1.0) and math.isclose(th, 84.0, abs_tol=2.5)


def test_ik_round_trip_wrist_on_the_ray_and_hand_along_it():
    from arm_pointer.mapping import solve_vertical_ik

    for psi in (-40.0, -15.0, 0.0, 20.0, 50.0):
        for reach in (0.05, 0.10, 0.14, 0.20):
            phis = solve_vertical_ik(psi, reach)
            (wx, wz), hand = _wrist_and_hand(phis)
            assert math.isclose(math.hypot(wx, wz), reach, abs_tol=1e-9)
            assert math.isclose(math.degrees(math.atan2(wz, wx)), psi, abs_tol=1e-6)
            assert math.isclose(hand, psi, abs_tol=1e-6)


def test_ik_unreachable_reach_is_clamped_not_failed():
    from arm_pointer.mapping import ARM, solve_vertical_ik

    far = solve_vertical_ik(10.0, 5.0)           # way beyond L1 + L2
    (wx, wz), _ = _wrist_and_hand(far)
    assert math.hypot(wx, wz) <= ARM.l1 + ARM.l2 and all(math.isfinite(v) for v in far)
    near = solve_vertical_ik(10.0, 0.0)          # inside the minimum reach |L1 - L2|
    (nx, nz), _ = _wrist_and_hand(near)
    assert math.hypot(nx, nz) >= abs(ARM.l1 - ARM.l2) - 1e-9


def test_ik_rejects_nan_and_inf():
    from arm_pointer.mapping import solve_vertical_ik

    assert solve_vertical_ik(float("nan"), 0.14) is None
    assert solve_vertical_ik(0.0, float("inf")) is None


def test_ik_uses_the_elbow_up_branch_only():
    from arm_pointer.mapping import ARM, segment_pitches, solve_vertical_ik

    for psi in (-40.0, 0.0, 50.0):
        tu, tf, _ = segment_pitches(*solve_vertical_ik(psi, 0.14))
        assert tf < tu          # forearm turned clockwise from the upper arm for every aiming direction


def test_ik_higher_target_raises_the_upper_arm_and_lowers_lift_raw():
    from arm_pointer.mapping import phi_to_raw, segment_pitches, solve_vertical_ik

    low, high = solve_vertical_ik(-20.0, 0.14), solve_vertical_ik(30.0, 0.14)
    assert segment_pitches(*high)[0] > segment_pitches(*low)[0]
    assert phi_to_raw(high[0], 2048) < phi_to_raw(low[0], 2048)    # raw increase = down on this arm


def test_pointing_pitch_geometry_and_clamp():
    from arm_pointer.mapping import pointing_pitch_deg

    assert math.isclose(pointing_pitch_deg(0.53, 0.0, 0.5, 0.03, 0.0, -90, 90), 45.0)     # 0.5 m ahead of the shoulder, 0.5 m up
    assert pointing_pitch_deg(0.5, 0.0, 5.0, 0.03, 0.0, -40, 50) == 50
    assert pointing_pitch_deg(0.5, 0.0, -5.0, 0.03, 0.0, -40, 50) == -40
    assert math.isclose(pointing_pitch_deg(0.0, 0.53, 0.5, 0.03, 0.0, -90, 90), 45.0)     # y counts as horizontal distance


def test_phi_raw_round_trip_and_sign():
    from arm_pointer.mapping import phi_to_raw, raw_to_phi

    for raw in (900, 1500, 2048, 3100):
        assert math.isclose(phi_to_raw(raw_to_phi(raw, 2000, 1), 2000, 1), raw)
        assert math.isclose(phi_to_raw(raw_to_phi(raw, 2000, -1), 2000, -1), raw)
    assert raw_to_phi(2048 + 1024, 2048) == 90.0          # raw up = positive (down-tilt) angle


def test_wrist_aim_hand_ray_passes_through_the_target():
    from arm_pointer.mapping import ARM, raw_to_phi, segment_pitches, wrist_aim_phi3, wrist_from_shoulder

    zero = (2400, 1840, 2070)
    phi1, phi2 = raw_to_phi(1500, zero[0]), raw_to_phi(1150, zero[1])          # the ready pose
    wx, wz = wrist_from_shoulder(phi1, phi2)
    for (x, y, z) in ((0.5, 0.0, 0.0), (0.45, 0.2, 0.15), (0.7, -0.1, 0.4), (0.4, 0.0, -0.1)):
        phi3, elev = wrist_aim_phi3(x, y, z, phi1, phi2, 0.0304, 0.0, -90, 90, side=-1)
        th = math.radians(segment_pitches(phi1, phi2, phi3)[2])                 # hand direction in the arm frame
        tx, tz = -math.hypot(x, y) - 0.0304 - wx, z - wz                        # wrist -> target, arm frame (target is behind)
        cross = math.cos(th) * tz - math.sin(th) * tx                           # 0 when the hand ray passes through the target
        assert abs(cross) < 1e-9 and math.cos(th) * tx + math.sin(th) * tz > 0  # and the target is in front of the hand


def test_wrist_aim_clamps_the_elevation_and_rejects_nan():
    from arm_pointer.mapping import wrist_aim_phi3

    _, up = wrist_aim_phi3(0.3, 0.0, 5.0, -40.0, -60.0, 0.03, 0.0, -30, 60, side=-1)
    _, down = wrist_aim_phi3(0.3, 0.0, -5.0, -40.0, -60.0, 0.03, 0.0, -30, 60, side=-1)
    assert up == 60 and down == -30
    assert wrist_aim_phi3(float('nan'), 0.0, 0.0, -40.0, -60.0, 0.03, 0.0, -30, 60) is None


def test_arm_angle_elevation_round_trip_for_both_sides():
    from arm_pointer.mapping import arm_angle_to_elevation, elevation_to_arm_angle

    for side in (1, -1):
        for e in (-60.0, -10.0, 0.0, 25.0, 80.0):
            assert math.isclose(arm_angle_to_elevation(elevation_to_arm_angle(e, side), side), e, abs_tol=1e-9)
    assert elevation_to_arm_angle(30.0, -1) == 150.0      # leaning back: up and toward the user


def test_pointing_pitch_back_side_adds_the_shoulder_offset():
    from arm_pointer.mapping import pointing_pitch_deg

    fwd = pointing_pitch_deg(0.5, 0.0, 0.5, 0.03, 0.0, -90, 90, side=1)     # horizontal 0.47
    back = pointing_pitch_deg(0.5, 0.0, 0.5, 0.03, 0.0, -90, 90, side=-1)    # horizontal 0.53
    assert back < fwd
