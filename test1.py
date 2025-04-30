#! /usr/bin/env python3

import sys
import os
import time
import threading
import numpy as np

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
 
def move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def load_dof_data(file_path):
    """Load action data from file using string manipulation"""
    dof_data = []
    
    with open(file_path, 'r') as f:
        for line in f:
            try:
                # 创建一个字典来存储解析后的数据
                data = {}
                
                # 提取world_vector数据
                world_start = line.find("'world_vector': array([") + len("'world_vector': array([")
                world_end = line.find("])", world_start)
                world_str = line[world_start:world_end]
                world_vector = [float(x.strip()) for x in world_str.split(',')]
                data['world_vector'] = world_vector
                
                # 提取rotation_delta数据
                rot_start = line.find("'rotation_delta': array([") + len("'rotation_delta': array([")
                rot_end = line.find("])", rot_start)
                rot_str = line[rot_start:rot_end]
                rotation_delta = [float(x.strip()) for x in rot_str.split(',')]
                data['rotation_delta'] = rotation_delta
                
                # 提取open_gripper数据
                grip_start = line.find("'open_gripper': array([") + len("'open_gripper': array([")
                grip_end = line.find("])", grip_start)
                grip_str = line[grip_start:grip_end]
                open_gripper = float(grip_str.strip())
                data['open_gripper'] = open_gripper
                
                dof_data.append(data)
            except Exception as e:
                print(f"Error parsing line: {e}")
                continue
    
    print(f"Successfully loaded {len(dof_data)} actions from file")
    return dof_data

def execute_dof_action(base, base_cyclic, dof_data):
    """Execute actions from DOF data"""
    
    print(f"Starting to execute {len(dof_data)} DOF actions...")
    
    # Get initial position as reference point
    feedback = base_cyclic.RefreshFeedback()
    initial_x = feedback.base.tool_pose_x
    initial_y = feedback.base.tool_pose_y
    initial_z = feedback.base.tool_pose_z
    initial_theta_x = feedback.base.tool_pose_theta_x
    initial_theta_y = feedback.base.tool_pose_theta_y
    initial_theta_z = feedback.base.tool_pose_theta_z
    
    success_count = 0
    
    # Execute each action in sequence
    for idx, data in enumerate(dof_data):
        print(f"Executing action {idx+1}/{len(dof_data)}")
        
        # Create action object
        action = Base_pb2.Action()
        action.name = f"DOF Action {idx+1}"
        action.application_data = ""
        
        # Extract vectors from dof_data
        world_vector = data['world_vector']
        rotation_delta = data['rotation_delta']
        gripper_state = data['open_gripper']
        
        # Set target pose
        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = initial_x + world_vector[0]
        cartesian_pose.y = initial_y + world_vector[1]
        cartesian_pose.z = initial_z + world_vector[2]
        cartesian_pose.theta_x = initial_theta_x + rotation_delta[0]
        cartesian_pose.theta_y = initial_theta_y + rotation_delta[1]
        cartesian_pose.theta_z = initial_theta_z + rotation_delta[2]
        
        # Create event to wait for action completion
        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(
            check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )
        
        # Execute the action
        base.ExecuteAction(action)
        
        # Wait for action to complete
        print("Waiting for movement to finish...")
        finished = e.wait(TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)
        
        if finished:
            print(f"Action {idx+1} completed successfully")
            success_count += 1
        else:
            print(f"Timeout on action {idx+1}")
            
        # Set gripper state
        gripper_action = Base_pb2.GripperCommand()
        gripper_action.mode = Base_pb2.GRIPPER_POSITION
        
        # Map gripper position from 0.0 (closed) to 1.0 (open)
        if gripper_state > 0.9:  # If value > 0.9, consider it open
            gripper_position = 1.0
            print("Opening gripper")
        else:
            gripper_position = 1.0 - gripper_state  # Map value to closed state
            print(f"Setting gripper position to {gripper_position}")
            
        finger = gripper_action.gripper.finger.add()
        finger.finger_identifier = 0
        finger.value = gripper_position
        base.SendGripperCommand(gripper_action)
        
        # Brief delay to complete action
        time.sleep(0.5)
    
    print(f"Completed {success_count}/{len(dof_data)} actions successfully")
    return success_count == len(dof_data)

def main():
    # Import utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # Load DOF data
        dof_file_path = "dof_example.txt"  # Ensure file path is correct
        dof_data = load_dof_data(dof_file_path)
        
        # Execute actions
        success = True
        success &= move_to_home_position(base)
        success &= execute_dof_action(base, base_cyclic, dof_data)
        
        # Return to home position when done
        success &= move_to_home_position(base)

        return 0 if success else 1

if __name__ == "__main__":
    exit(main())