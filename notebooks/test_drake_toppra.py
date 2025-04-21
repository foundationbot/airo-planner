import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from pydrake.all import (
    MultibodyPlant, SceneGraph, DiagramBuilder, RigidTransform,
    RotationMatrix, CoulombFriction, Box, Cylinder, Meshcat,
    PiecewisePolynomial, PiecewisePose
)
from pydrake.visualization import AddDefaultVisualization
from pydrake.multibody.optimization import Toppra
import ompl.base as ob
import ompl.geometric as og
from pydrake.multibody.tree import UnitInertia, SpatialInertia, RigidBody
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from ompl import util as ou



# === Scene setup ===
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

# Add cube

cube_shape = Box(0.1, 0.1, 0.1)
mass = 1.0
inertia = UnitInertia.SolidBox(0.1, 0.1, 0.1)
spatial_inertia = SpatialInertia(mass=mass, p_PScm_E=[0, 0, 0], G_SP_E=inertia)
cube_body = plant.AddRigidBody("cube", spatial_inertia)
plant.RegisterCollisionGeometry(cube_body, RigidTransform(), cube_shape, "cube_collision", CoulombFriction(0.9, 0.8))
plant.RegisterVisualGeometry(cube_body, RigidTransform(), cube_shape, "cube_visual", [1, 0, 0, 1])

# Add cylinder obstacle
cylinder = Cylinder(0.2, 1.0)
cyl_pose = RigidTransform([0.5, 0, 0.5])
plant.RegisterCollisionGeometry(plant.world_body(), cyl_pose, cylinder, "cylinder_collision", CoulombFriction(0.9, 0.8))
plant.RegisterVisualGeometry(plant.world_body(), cyl_pose, cylinder, "cylinder_visual", [0, 0, 1, 1])

plant.Finalize()

# === Meshcat ===
meshcat = Meshcat()
AddDefaultVisualization(builder, meshcat=meshcat)
diagram = builder.Build()
context = diagram.CreateDefaultContext()
plant_context = plant.GetMyContextFromRoot(context)

# === OMPL SE(3) planner ===
space = ob.SE3StateSpace()
bounds = ob.RealVectorBounds(3)
bounds.setLow(0, -1.0)
bounds.setLow(1, -1.0)
bounds.setLow(2,  0.0)

bounds.setHigh(0, 1.5)
bounds.setHigh(1, 1.5)
bounds.setHigh(2, 1.5)

space.setBounds(bounds)

si = ob.SpaceInformation(space)
si.setStateValidityChecker(ob.AllValidStateValidityChecker(si))
si.setup()

start = ob.State(space)
start().setXYZ(0, 0, 0)
start().rotation().setIdentity()

goal = ob.State(space)
goal().setXYZ(1.0, 0.0, 1.0)
goal().rotation().setIdentity()

pdef = ob.ProblemDefinition(si)
pdef.setStartAndGoalStates(start, goal)
planner = og.RRTConnect(si)
planner.setProblemDefinition(pdef)
planner.setup()
solved = planner.solve(5.0)
assert solved, "OMPL failed to find a solution"

# === Extract SE(3) path ===
states = pdef.getSolutionPath().getStates()
positions = np.array([[s.getX(), s.getY(), s.getZ()] for s in states])
quats = np.array([[s.rotation().x, s.rotation().y, s.rotation().z, s.rotation().w] for s in states])
times = np.linspace(0, 1, len(positions))

# === Interpolate orientation using Slerp ===
rots = R.from_quat(quats)
slerp = Slerp(times, rots)

interp_times = np.linspace(0, 1, 200)
interp_pos = np.array([np.interp(interp_times, times, positions[:, i]) for i in range(3)]).T
interp_rots = slerp(interp_times)

# === Create interpolated path
q_samples = []
for pos, rot in zip(interp_pos, interp_rots):
    quat = rot.as_quat()  # [x, y, z, w]
    quat_wxyz = [quat[3], quat[0], quat[1], quat[2]]  # Reorder for Drake
    q = list(pos) + quat_wxyz
    q_samples.append(q)

q_samples = np.array(q_samples).T  # shape (7, N)
q_traj = PiecewisePolynomial.FirstOrderHold(interp_times, q_samples)

# === Apply Toppra
vlim = np.array([1.0, 1.0, 1.0])
alim = np.array([2.0, 2.0, 2.0])
# Gridpoints for evaluating the path
s_grid = np.linspace(0, 1, 100)
toppra = Toppra(q_traj, plant, s_grid)

# Set limits (these are per joint — here: x, y, z of cube in world)
toppra.SetJointVelocityLimits(np.array([1.0, 1.0, 1.0]))
toppra.SetJointAccelerationLimits(np.array([2.0, 2.0, 2.0]))

# Set initial/final velocity
toppra.SetInitialVelocity(np.random.uniform(-0.5, 0.5, 3))
toppra.SetFinalVelocity(np.zeros(3))

# Solve!
timed_traj = toppra.SolvePathParameterization()

# === Retimed orientation via Slerp
t_samples = np.linspace(timed_traj.start_time(), timed_traj.end_time(), 200)
xyz_retimed = np.array([timed_traj.value(t) for t in t_samples]).T
rot_retimed = slerp(np.interp(t_samples, interp_times, interp_times))

# === Final SE(3) trajectory
poses = [RigidTransform(RotationMatrix(r.as_matrix()), p) for r, p in zip(rot_retimed, xyz_retimed)]
pose_traj = PiecewisePose.MakeLinear(poses, t_samples)

# === Animate in Meshcat
for t in t_samples:
    pose = pose_traj.value(t)
    plant.SetFreeBodyPose(plant_context, cube_body, pose)
    diagram.ForcedPublish(context)
