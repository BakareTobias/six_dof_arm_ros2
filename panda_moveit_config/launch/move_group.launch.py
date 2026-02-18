from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_moveit_config").robot_description(file_path="config/panda.urdf.xacro").to_moveit_configs()
    
    # Debug: Print loaded kinematics
    print("DEBUG: Loaded Kinematics:", moveit_config.robot_description_kinematics)
    
    return generate_move_group_launch(moveit_config)
