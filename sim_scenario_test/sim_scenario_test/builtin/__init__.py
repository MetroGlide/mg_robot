"""組み込みの型 (action / trigger / expectation / driver / backend / waypoint 形式)。

import するだけで DEFAULT_REGISTRY に登録される。
"""
from sim_scenario_test.builtin import actions  # noqa: F401
from sim_scenario_test.builtin import expectations  # noqa: F401
from sim_scenario_test.builtin import triggers  # noqa: F401
from sim_scenario_test.builtin import waypoint_formats  # noqa: F401
from sim_scenario_test.drivers import nav2_goals  # noqa: F401
from sim_scenario_test.sim import gazebo_fortress  # noqa: F401
