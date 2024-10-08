###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time
import threading

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2


# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check

def set_gripper(base, position):
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()

    # Close the gripper with position increments
    print("Performing gripper test in position...")
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    finger.value = position
    print(f"Going to position {position}")
    base.SendGripperCommand(gripper_command)


def get_gripper(base):
    gripper_request = Base_pb2.GripperRequest()
    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
    if len (gripper_measure.finger):
        print(f"Current position is : {gripper_measure.finger[0].value}")
        return gripper_measure.finger[0].value
    return None
def example_angular_action_movement(base, angles):
    print(f"Starting angular action movement with angles {angles}...")
    action = Base_pb2.Action()
    action.name = "Angular action movement"
    action.application_data = ""

    # Set the target joint angles
    actuator_count = base.GetActuatorCount()
    if actuator_count.count != len(angles):
        print(f"Bad length for angles. Actuators count: {actuator_count.count}, Angles provided: {len(angles)}")
        return False

    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = angles[joint_id]

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Angular movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def pick_and_move_with_angular_action(base, pickup_angles, dropoff_angles):
    # Move to pickup position with specified joint angles
    success = example_angular_action_movement(base, pickup_angles)
    if not success:
        print("Failed to reach pickup position")
        return False

    # Close gripper to grab block
    print("Closing gripper to grab block...")
    set_gripper(base, 1.0)
    time.sleep(2)  # Give time for gripper to close

    # Lift the block by changing the angles slightly
    lift_angles = pickup_angles[:]
    lift_angles[2] -= 20  # Raise the arm at the elbow joint (for example)
    success = example_angular_action_movement(base, lift_angles)
    if not success:
        print("Failed to lift the block")
        return False

    # Move to drop-off position
    print("Moving to drop-off position...")
    success = example_angular_action_movement(base, dropoff_angles)
    if not success:
        print("Failed to reach drop-off position")
        return False

    # Open gripper to release block
    print("Opening gripper to release block...")
    set_gripper(base, 0.0)
    time.sleep(2)  # Give time for gripper to open

    # Move back up after releasing
    return True

def main():
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)

        # Move arm to home position first
        success = example_move_to_home_position(base)

        # Define angular positions for pickup and drop-off (in degrees)
        # These angles will need to be adjusted based on the robot's configuration.
        pickup_angles = [-45, 50, -45, 45, 45, 90]   # Example angles to move to the block
        dropoff_angles = [60, 0, -30, 0, 90, 0]  # Example angles to move to the drop-off point

        # Perform the pickup and drop-off
        success = pick_and_move_with_angular_action(base, pickup_angles, dropoff_angles)
        if success:
            print("Successfully picked up and dropped off the block")

        # Move back to home position
        success &= example_move_to_home_position(base)

        return 0 if success else 1

if __name__ == "__main__":
    exit(main())
