import pybullet as p
import pybullet_data
import time

# Connect to the PyBullet physics server
physicsClient = p.connect(p.GUI)  # Use p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set the data path for built-in assets

# Load the ground plane
p.loadURDF("plane.urdf")

# Load the UR5 robot arm
robot = p.loadURDF("ur5/ur5.urdf", [0, 0, 0], useFixedBase=True)

# Set gravity
p.setGravity(0, 0, -9.81)

# Define the end effector link index (usually the last link)
end_effector_link_index = 5  # For UR5, the end effector is link 5

# Define a target position for the end effector
target_position = [0.5, 0.5, 0.5]  # Target position in world coordinates

# Set the simulation time step and enable real-time simulation
p.setTimeStep(1.0 / 240.0)  # 240 Hz simulation step
p.setRealTimeSimulation(1)

# Main simulation loop
for _ in range(1000):  # Run for 1000 steps
    # Compute inverse kinematics to get joint positions for the target end effector position
    joint_positions = p.calculateInverseKinematics(
        robot,  # Robot ID
        end_effector_link_index,  # End effector link index
        target_position,  # Target position
    )

    # Set the joint positions for the robot
    for i in range(len(joint_positions)):
        p.setJointMotorControl2(
            robot,  # Robot ID
            i,  # Joint index
            p.POSITION_CONTROL,  # Control mode
            targetPosition=joint_positions[i],  # Target joint position
        )

    # Step the simulation
    p.stepSimulation()

    # Optional: Add a small delay to slow down the simulation for visualization
    time.sleep(1.0 / 240.0)

# Disconnect from the physics server
p.disconnect()