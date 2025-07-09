import numpy as np
from airo_drake import DualArmScene, add_floor, add_manipulator, add_meshcat, add_wall, finish_build, SingleArmScene
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.planning import RobotDiagramBuilder, SceneGraphCollisionChecker
import airo_models


def add_humanoid_scene(
    robot_diagram_builder: RobotDiagramBuilder,
) -> tuple[ModelInstanceIndex, ModelInstanceIndex]:
    plant = robot_diagram_builder.plant()
    parser = robot_diagram_builder.parser()
    parser.SetAutoRenaming(True)

    add_floor(robot_diagram_builder, x_size=0.4, y_size=0.4)

    # Add your humanoid robot (with both arms in a single URDF)
    parser.package_map().Add(
    "phantom_description",
    "/home/ananth/foundation/airo-planner/notebooks/phantom_description",
    )
    parser.package_map().Add(
    "ability_hand_description",
    "/home/ananth/foundation/airo-planner/notebooks/ability_hand_description"
    )
    idxs = parser.AddModels("./phantom_description/urdf/20241024_Alex_Nubs.urdf")
    humanoid_index = idxs[0]
    print(idxs)

    world_frame = plant.world_frame()
    base_frame = plant.GetFrameByName("Pelvis", humanoid_index)
    plant.WeldFrames(world_frame, base_frame)

    # Optionally, add walls or other scene elements here

    # Return the model instance index (or joint indices) for left and right arms
    # You may need to get joint indices/names for each arm
    return humanoid_index



robot_diagram_builder = RobotDiagramBuilder()

meshcat = add_meshcat(robot_diagram_builder)
meshcat.SetCameraPose([-1.5, 0, 1.0], [0, 0, 0])

arm_y = 0.45
X_W_L = RigidTransform(rpy=RollPitchYaw([0, 0, np.pi / 2]), p=[0, arm_y, 0])
X_W_R = RigidTransform(rpy=RollPitchYaw([0, 0, np.pi / 2]), p=[0, -arm_y, 0])


# (arm_left_index, arm_right_index), (gripper_left_index, gripper_right_index) = add_cloth_competition_dual_ur5e_scene(
#     robot_diagram_builder, X_W_L, X_W_R
# )
humanoid_index = add_humanoid_scene(robot_diagram_builder)
robot_diagram, context = finish_build(robot_diagram_builder, meshcat)

# scene = DualArmScene(robot_diagram, arm_left_index, arm_right_index, gripper_left_index, gripper_right_index, meshcat)
# scene = DualArmScene(
#     robot_diagram=robot_diagram,
#     arm_left_index=humanoid_index,
#     arm_right_index=humanoid_index,
#     gripper_left_index=None,  # or set if you have gripper sub-instances
#     gripper_right_index=None,
#     meshcat=meshcat,
# )

scene = SingleArmScene(
    robot_diagram=robot_diagram,
    arm_index=humanoid_index,
    gripper_index=None,  # or set if you have gripper sub-instances
    meshcat=meshcat,
)

collision_checker = SceneGraphCollisionChecker(
    model=scene.robot_diagram,
    robot_model_instances=[
        scene.arm_index,
    ],
    edge_step_size=0.125,  # Arbitrary value: we don't use the CheckEdgeCollisionFree
    env_collision_padding=0.005,
    self_collision_padding=0.005,
)

scene.robot_diagram.plant().num_positions()
collision_checker.CheckConfigCollisionFree(np.zeros(18))

from pydrake.multibody.tree import JointIndex
plant = scene.robot_diagram.plant()
# joint_types = [plant.get_joint(JointIndex(j)).type_name()  for j in range(plant.num_joints())]
joint_names = [plant.get_joint(JointIndex(j)).name() for j in range(plant.num_joints()) if plant.get_joint(JointIndex(j)).type_name() == "revolute"]
print(joint_names)
left_arm_idxs = [idx for idx in range(len(joint_names)) if "Left" in joint_names[idx]]
right_arm_idxs = [idx for idx in range(len(joint_names)) if "Right" in joint_names[idx]]
print("Left arm joint indices:", left_arm_idxs)
print("Right arm joint indices:", right_arm_idxs)


home_joints_left = np.array([0.5, 0.2, 0.0, -1.5, 0.0, 0.0, 0.0])
home_joints_right = np.array([0.5, -0.2, 0, -1.5, 0.0, 0.0, 0.0])

home_joints = np.zeros(18)
home_joints[left_arm_idxs] = home_joints_left
home_joints[right_arm_idxs] = home_joints_right

print("home_joints: ", home_joints)


collision_checker.CheckConfigCollisionFree(home_joints)

plant = scene.robot_diagram.plant()
plant_context = plant.GetMyContextFromRoot(context)

plant.SetPositions(plant_context, home_joints)
scene.robot_diagram.ForcedPublish(context)  # updates the meshcat visualization


from airo_planner import DualArmOmplPlanner

# These joint bounds are specifically for UR5e mounted on a table.
# joint_bounds_lower = np.deg2rad([-360, -195, -160, -360, -360, -360])
# joint_bounds_upper = np.deg2rad([360, 15, 160, 360, 360, 360])
joint_bounds = None

def is_state_valid_fn_dual_arm(joints):
    print(joints)
    q = np.zeros(18)
    joints_left = joints[:7]
    joints_right = joints[7:]
    q[left_arm_idxs] = joints_left
    q[right_arm_idxs] = joints_right
    return collision_checker.CheckConfigCollisionFree(q)

planner = DualArmOmplPlanner(
    is_state_valid_fn=is_state_valid_fn_dual_arm,
    # inverse_kinematics_left_fn=inverse_kinematics_left_fn,
    # inverse_kinematics_right_fn=inverse_kinematics_right_fn,
    joint_bounds_left=joint_bounds,
    joint_bounds_right=joint_bounds,
    degrees_of_freedom_left=7,
    degrees_of_freedom_right=7,
)

tangled_joints_left = np.deg2rad([90, 90, -90, -90, 90, 0, 0])
tangled_joints_right = np.deg2rad([-136, -116, -110, -133, 40, 0, 0])
tangled_joints = np.zeros(18)
tangled_joints[left_arm_idxs] = tangled_joints_left
tangled_joints[right_arm_idxs] = tangled_joints_right
# tangled_joints = np.concatenate([tangled_joints_left, tangled_joints_right])

plant.SetPositions(plant_context, tangled_joints)
scene.robot_diagram.ForcedPublish(context)  # updates the meshcat visualization

path = planner.plan_to_joint_configuration(
    tangled_joints_left, tangled_joints_right, home_joints_left, home_joints_right
)
print(path.shape)
