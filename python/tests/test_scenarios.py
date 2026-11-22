import numpy as np
from cbf_robot import scenarios as S, metrics as M
from cbf_robot.controller import QPParams
from cbf_robot.references import CircleRef


def test_circular_safe_and_close_to_circle():
    r = S.run_circular(S.CircularConfig(T=30.0))
    assert M.min_h(r) > -5e-3                    # nenhum obstáculo invadido
    rms, mx = M.distance_to_circle(r)
    assert rms < 0.2 and mx < 0.5


def test_goal_reaches_pose():
    r = S.run_goal(S.GoalConfig(G=(3., 4.), start=(0., 0., 1.), obstacles=((1.5, 2), (3, 3), (4, 4.6)), T=30.0))
    pos, ori = M.goal_errors(r, (3., 4.), np.pi / 4)
    assert pos < 0.015 and ori < 0.01            # < 1,5 cm e < 0,01 rad
    assert M.min_h(r) > -5e-3


def test_line_deformed_is_safe():
    r = S.run_line(S.LineConfig(T=60.0))
    assert M.min_h(r) > -5e-3
    assert M.distance_to_line(r)[1] < 0.5


def test_guard_ignores_far_obstacle():
    ref = CircleRef(guard=True)
    far = ((0.0, 1.0),)                          # a 1 m do centro: não bloqueia o círculo de raio 2
    d = ref.deformed(7.0, far, 0.3)
    xd0, yd0, _ = ref.original(7.0)
    assert np.hypot(d.xd - xd0, d.yd - yd0) < 1e-9
