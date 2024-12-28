from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("bcn3d_moveo", package_name="bcn3d_moveo_moveit").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)