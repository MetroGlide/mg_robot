"""mg_navigation.localization_supervisor package

自己位置推定 (AMCL + GNSS + オドメトリの EKF 融合) を監督し、AMCL がずれたときに
EKF から切り離して EKF の姿勢で復旧する。判定 (checks / scan_check) と状態遷移 (state_machine) は
ROS に依存しない。ノードは node.py。
"""
