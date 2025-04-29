#!/usr/bin/env python3

import os
import sys
import time
import threading
import ast

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.GripperCommandClientRpc import GripperCommandClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, GripperCommand_pb2

# 最大等待时间（秒）
TIMEOUT_DURATION = 20

def check_for_end_or_abort(e):
    def check(notification, e=e):
        print("Event: " + Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event in [Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT]:
            e.set()
    return check

def execute_cartesian_delta(base, base_cyclic, delta, rotation, timeout=TIMEOUT_DURATION):
    feedback = base_cyclic.RefreshFeedback()
    action = Base_pb2.Action()
    action.name = "Cartesian delta move"

    pose = action.reach_pose.target_pose
    pose.x = feedback.base.tool_pose_x + delta[0]
    pose.y = feedback.base.tool_pose_y + delta[1]
    pose.z = feedback.base.tool_pose_z + delta[2]
    pose.theta_x = feedback.base.tool_pose_theta_x + rotation[0]
    pose.theta_y = feedback.base.tool_pose_theta_y + rotation[1]
    pose.theta_z = feedback.base.tool_pose_theta_z + rotation[2]

    e = threading.Event()
    handle = base.OnNotificationActionTopic(check_for_end_or_abort(e), Base_pb2.NotificationOptions())
    base.ExecuteAction(action)
    e.wait(timeout)
    base.Unsubscribe(handle)

def set_gripper_position(gripper, value):
    command = GripperCommand_pb2.GripperCommand()
    finger = command.gripper_opening.command
    finger.value = value * 100  # 转换为百分比（0~100）
    gripper.SendGripperCommand(command)

def load_dof_actions(file_path):
    actions = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.strip():
                data = ast.literal_eval(line.strip())
                actions.append(data)
    return actions

def main():
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    args = utilities.parseConnectionArguments()

    # 建立 TCP 连接
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        gripper = GripperCommandClient(router)

        # 可选：先移动到 Home 位置
        print("Moving to Home position...")
        home_action = Base_pb2.RequestedActionType()
        home_action.action_type = Base_pb2.REACH_JOINT_ANGLES
        actions = base.ReadAllActions(home_action)
        for a in actions.action_list:
            if a.name == "Home":
                base.ExecuteActionFromReference(a.handle)
                time.sleep(5)

        # 加载动作指令
        actions = load_dof_actions("dof_example.txt")
        for i, action in enumerate(actions):
            print(f"\n== Step {i+1} ==")
            delta = action["world_vector"]
            rotation = action["rotation_delta"]
            gripper_value = action["open_gripper"][0]

            # 执行运动
            execute_cartesian_delta(base, base_cyclic, delta, rotation)

            # 控制夹爪
            set_gripper_position(gripper, gripper_value)

            time.sleep(1)

if __name__ == "__main__":
    exit(main())