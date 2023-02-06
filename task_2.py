#!/usr/bin/env python3

import json
import numpy as np
import pybullet as p
import time
import random
import math

MAX_ROTATION_ERROR = 0.01
MAX_DISPLACEMENT_ERROR = 5e-3
LINEAR_VELOCITY_THRESHOLD = 0.01
ANGULAR_VELOCITY_THRESHOLD = 0.1
GAP_Y = 0.31
TIMECAP = 10

with open("test-cases.json", "r") as read_file:
    test_cases_json = json.load(read_file)
FIXED_TEST_CASES = test_cases_json["test_cases"]
# N_TESTS = len(FIXED_TEST_CASES) + 10  # ten random test cases
N_TESTS = 1


def PrintArray(arr, fmt='%8.2f'):
    np.savetxt('/dev/stdout', arr, fmt)
    

def execute_control(filler_id, robot_id, target_xy):
    # EDIT ONLY BELOW THIS LINE

    if not hasattr(execute_control, 'robot'):

        print('target_xy', target_xy)

        class Robot(object):

            def __init__(self):
                self._actions = self._Actions()
            
            def x(self):
                return p.getJointState(robot_id, 1)[0]

            def y(self):
                return p.getJointState(robot_id, 0)[0]

            def yaw(self):
                # along X: yaw = 0
                # along Y: yaw = np.pi / 2
                _, orientation = p.getBasePositionAndOrientation(filler_id)
                return p.getEulerFromQuaternion(orientation)[2]

            def NextAction(self):
                next(self._actions)
                
            def _Actions(self):
                yield from self._RotateTo(yaw=np.pi/2)
                print('### 0')
                # yield from self._GoToY(target_xy[1])
                # print('### 1')
                # yield from self._GoToX(target_xy[0])
                # print('### 2')
                time.sleep(3600)

            def _GoToX(self, x):
                while np.abs(self.x() - x) > MAX_DISPLACEMENT_ERROR:
                    print('yaw = %10.6f' % self.yaw())
                    # print('position x=%8.2f y=%8.2f' % (self.x(), self.y()))
                    p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, targetPosition=x, maxVelocity=1, force=1)
                    # p.setJointMotorControl2(robot_id, 1, p.VELOCITY_CONTROL, targetVelocity=direction, force=1)
                    yield

            def _GoToY(self, y):
                while np.abs(self.y() - y) > MAX_DISPLACEMENT_ERROR:
                    print('yaw = %10.6f' % self.yaw())
                    # print('position x=%8.2f y=%8.2f' % (self.x(), self.y()))
                    p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, targetPosition=y, maxVelocity=1)
                    # p.setJointMotorControl2(robot_id, 0, p.VELOCITY_CONTROL, targetVelocity=direction, force=1)
                    yield

            def _RotateTo(self, yaw):
                while np.abs(self.yaw() - yaw) > MAX_ROTATION_ERROR:
                    print('yaw = %10.6f' % self.yaw())
                    p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=yaw)
                    yield

        execute_control.robot = Robot()
        
    execute_control.robot.NextAction()
    
    
    # maxVelocity
    # 0 - move along Y, 1 - move along X
    # p.setJointMotorControl2(robot_id, 0, p.VELOCITY_CONTROL, targetVelocity=1, force=1)

    pass
    # EDIT ONLY ABOVE THIS LINE


def build_world(world_n=None):
    p.resetSimulation()
    p.resetDebugVisualizerCamera(2, 50, -35, [0, 0, 0])

    p.setGravity(0, 0, -9.81)
    robot = p.loadURDF("robot.urdf")
    if world_n is not None and world_n < len(FIXED_TEST_CASES):
        test_case = FIXED_TEST_CASES[world_n]
        bridge_x = test_case["bridge_x"]
        filler_orientation = test_case["orientation"]
        randomly_generated = False
    else:
        bridge_x = (0.135 + 0.5 * random.random()) * [-1, 1][random.randrange(2)]
        filler_orientation = [0, 0, random.random() * 2 * math.pi]
        randomly_generated = True
    print("------------------------------------------\n" * 2, end="")
    print("Running simulation:")
    print(f" - bridge at x: {bridge_x}")
    print(f" - filler orientantion: {filler_orientation}")
    print(f" - randomly generated: {randomly_generated}")
    bridge = p.loadURDF("bridge.urdf", [bridge_x, 0, 0])
    filler = p.loadURDF(
        "filler.urdf",
        [0, 0, 0.12],
        p.getQuaternionFromEuler(filler_orientation),
    )
    p.changeDynamics(filler, -1, lateralFriction=5)
    p.changeDynamics(robot, 2, lateralFriction=5)

    return robot, bridge_x, filler


def get_filler_status(filler, bridge_x):
    filler_pose = p.getBasePositionAndOrientation(filler)
    roll_error, pitch_error, yaw = p.getEulerFromQuaternion(filler_pose[1])
    yaw_error = min(abs(yaw - math.pi / 2), abs(yaw + math.pi / 2))
    d_errors = [abs(filler_pose[0][0] - bridge_x), abs(filler_pose[0][1] - GAP_Y)]
    orientation_errors = [abs(roll_error), abs(pitch_error), yaw_error]

    linear_vs, angular_vs = p.getBaseVelocity(filler)
    linear_v = np.linalg.norm(linear_vs)
    angular_v = np.linalg.norm(angular_vs)

    return max(d_errors), orientation_errors, linear_v, angular_v


def show_msg(msg_id, text, rgb=[0, 0, 1]):
    """Show message in GUI

    Replace old msg if exists
    """
    MSG_POS = [0.5, -0.5, 0.15]
    if msg_id is not None:
        msg_id = p.addUserDebugText(
            text,
            MSG_POS,
            textColorRGB=rgb,
            textSize=2,
            replaceItemUniqueId=msg_id,
        )
    else:
        msg_id = p.addUserDebugText(text, MSG_POS, textColorRGB=rgb, textSize=2)
    return msg_id


def fill_the_gap(filler, robot, bridge_x):
    sim_step = 0

    msg_id = None

    already_yellow = False
    already_purple = True

    while True:
        current_time = sim_step / 240
        if sim_step % 24 < 1e-2:
            time_left = TIMECAP - current_time
            msg_id = show_msg(msg_id, f"Time left: {round(time_left, 1)} s", [0, 0, 1])

        execute_control(filler, robot, [bridge_x, GAP_Y])

        current_d_error, orientation_errors, linear_v, angular_v = get_filler_status(
            filler, bridge_x
        )

        correct_placement = current_d_error < MAX_DISPLACEMENT_ERROR
        correct_orientation = max(orientation_errors) < MAX_ROTATION_ERROR
        moving = (
            linear_v > LINEAR_VELOCITY_THRESHOLD
            or angular_v > ANGULAR_VELOCITY_THRESHOLD
        )

        # print('current_d_error = %10.6f' % current_d_error)
        # print('orientation_errors = ', orientation_errors)
        # print('linear_v = %10.6f' % linear_v)
        # print('angular_v = %10.6f' % angular_v)

        if correct_placement and correct_orientation and not moving:
            p.changeVisualShape(filler, -1, rgbaColor=[0, 1, 0, 1])
            time.sleep(3)
            return True, current_time
        elif correct_placement and correct_orientation:
            if not already_yellow:
                p.changeVisualShape(filler, -1, rgbaColor=[1, 1, 0, 1])
                already_yellow = True
                already_purple = False
            p.stepSimulation()
            sim_step = sim_step + 1
            time.sleep(0.01)
        elif current_time > TIMECAP:
            p.changeVisualShape(filler, -1, rgbaColor=[1, 0, 0, 1])
            time.sleep(3)
            return False, current_time
        else:
            if not already_purple:
                p.changeVisualShape(filler, -1, rgbaColor=[0.8, 0.2, 0.8, 1])
                already_purple = True
                already_yellow = False
            p.stepSimulation()
            sim_step = sim_step + 1
            time.sleep(0.01)


def test(test_index):            
    robot, bridge_x, filler = build_world(test_index)

    for _ in range(30):
        p.stepSimulation()
        time.sleep(0.01)

    succeded, current_time = fill_the_gap(filler, robot, bridge_x)
    return succeded, current_time
            

def main():
    p.connect(p.GUI)

    n_succes = 0
    total_time_succes = 0

    import sys
    test(int(sys.argv[1]))

    # for test_index in range(N_TESTS):
    #     succeded, current_time = test(test_index)
    #     if succeded:
    #         n_succes = n_succes + 1
    #         total_time_succes = total_time_succes + current_time

    print(f"Tested solution effectiveness: {n_succes / N_TESTS}")
    print(f"Average time to reach succesfull solution: {total_time_succes / n_succes}")


if __name__ == "__main__":
    main()
