import group_interface

interface = group_interface.GroupInterface()

print("")
print("MoveIt group: arm")
print("Current joint states (radians): {}".format(interface.get_joint_state("arm")))
print("Current joint states (degrees): {}".format(interface.get_joint_state("arm", degrees=True)))
print("Current cartesian pose: {}".format(interface.get_cartesian_pose("arm")))

print("")
print("Planning group: arm")

print("  |-- Reaching named pose...")
interface.reach_named_pose("arm", "home")
try:
    input("Wait for key")
except:
    pass

print("  |-- Reaching cartesian pose...")
pose = interface.get_cartesian_pose("arm")
pose.position.z -= 0.10 
interface.reach_cartesian_pose("arm", pose)
try:
    input("Wait for key")
except:
    pass
print("  |-- Reaching joint state (radians)...")
interface.reach_joint_state("arm", [0, 0, 0, -1.57, 0, 2.0, 0])
try:
    input("Wait for key")
except:
    pass
print("  |-- Reaching joint state (degrees)...")
interface.reach_joint_state("arm", [0, 0, 0, 90, 0, 0, 0], degrees=True)
try:
    input("Wait for key")
except:
    pass


print("")
print("Planning group: gripper")

print("  |-- Reaching named pose...")
interface.reach_named_pose("gripper", "open")
try:
    input("Wait for key")
except:
    pass

print("  |-- Reaching named pose...")
interface.reach_named_pose("gripper", "close")
try:
    input("Wait for key")
except:
    pass


# print("")
# print("FollowJointTrajectory action")

# interface.setup_follow_joint_trajectory(name='/panda/gripper_controller/follow_joint_trajectory', 
#                                         joints=["panda_finger_joint1", "panda_finger_joint2"])

# print("  |-- Apply position [0.1, 0.1]")
# interface.apply_follow_joint_trajectory(positions=[0.1, 0.1])

# try:
#     input("Wait for key")
# except:
#     pass

# print("  |-- Apply position [0.0, 0.0]")
# interface.apply_follow_joint_trajectory(positions=[0.0, 0.0])