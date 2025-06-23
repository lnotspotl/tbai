#!/usr/bin/env python3

import sys
sys.path.insert(0, "/home/kuba/Documents/tbai2/build/tbai_python")

import tbai_python
import numpy as np
import time
import threading
import pybullet as p
import pybullet_data

from joystick import UIController

import mpac_logging
from mpac_logging.rerun.robot_logger import RobotLogger
from mpac_logging.rerun.utils import rerun_initialize, rerun_store


from tbai_python import (
    StateSubscriber,
    CommandPublisher,
    ChangeControllerSubscriber,
    ReferenceVelocity,
    ReferenceVelocityGenerator,
)

from tbai_python.rotations import rpy2quat, quat2mat, mat2rpy, mat2ocs2rpy, ocs2rpy2quat, rpy2mat, mat2aa

kp = 30
kd = 0.5
desired_joint_angles = np.zeros(12)
running = True

joint2idx = {
    "LF_HAA": 0,
    "LF_HFE": 1,
    "LF_KFE": 2,
    "RF_HAA": 3,
    "RF_HFE": 4,
    "RF_KFE": 5,
    "LH_HAA": 6,
    "LH_HFE": 7,
    "LH_KFE": 8,
    "RH_HAA": 9,
    "RH_HFE": 10,
    "RH_KFE": 11,
}

def pd_controller(q_current, q_desired, v_desired, v_current, kp, kd) -> np.ndarray:
    return kp * (q_desired - q_current) + kd * (v_desired - v_current)

class PyBulletStateSubscriber(StateSubscriber):
    def __init__(self, robot_id):
        super().__init__()
        self.initialized = False
        self.current_state = np.zeros((36,), dtype=np.float64)
        self.robot_id = robot_id
        self.last_time = time.time()
        self.last_orientation = np.eye(3)
        self.last_position = np.zeros(3)
        self.last_joint_angles = np.zeros(12)
        self.last_yaw = 0.0
        self.first_update = True
        self.needs_update = True

    def waitTillInitialized(self):
        self.initialized = True

    def getLatestRbdStateViz(self):
        return self.current_state

    def getLatestRbdState(self):
        if not self.needs_update:
            return self.current_state
        self.needs_update = False

        current_time = time.time()
        dt = current_time - self.last_time

        # Get base orientation and position
        base_pos, base_quat = p.getBasePositionAndOrientation(self.robot_id)
        base_quat = np.array([base_quat[0], base_quat[1], base_quat[2], base_quat[3]])  # wxyz to xyzw
        R_world_base = quat2mat(base_quat)
        R_base_world = R_world_base.T

        # Get RPY angles
        rpy = mat2ocs2rpy(R_world_base, self.last_yaw)
        self.last_yaw = rpy[2]

        # Base position in world frame
        base_position = np.array(base_pos)

        if self.first_update:
            self.last_orientation = R_base_world
            self.last_position = base_position
            self.first_update = False

        # Get base velocities
        base_vel, base_ang_vel = p.getBaseVelocity(self.robot_id)
        
        # Convert to base frame
        linear_velocity_base = R_base_world @ np.array(base_vel)
        angular_velocity_base = R_base_world @ np.array(base_ang_vel)

        # Get joint states
        joint_angles = []
        joint_velocities = []
        for joint_name in joint2idx.keys():
            joint_state = p.getJointState(self.robot_id, joint_name_to_id[joint_name])
            joint_angles.append(joint_state[0])
            joint_velocities.append(joint_state[1])

        joint_angles = np.array(joint_angles)
        joint_velocities = np.array(joint_velocities)

        # Update state vector
        self.current_state[0:3] = rpy  # Base orientation (RPY)
        self.current_state[3:6] = base_position  # Base position
        self.current_state[6:9] = angular_velocity_base  # Base angular velocity
        self.current_state[9:12] = linear_velocity_base  # Base linear velocity
        self.current_state[12:24] = joint_angles  # Joint positions
        self.current_state[24:36] = joint_velocities  # Joint velocities

        # Update last values
        self.last_orientation = R_base_world
        self.last_position = base_position
        self.last_time = current_time

        return self.current_state

    def getLatestRbdStamp(self):
        return time.time()

    def getContactFlags(self):
        return [False] * 4


class RerunLoggerNode:
    def __init__(self, state_subscriber: StateSubscriber, freq = 10):
        rerun_initialize("simple_robot_example", spawn=False)
        self.robot_logger = RobotLogger.from_zoo("go2_description")
        self.state_subscriber = state_subscriber
        self.freq = freq
        self.last_time = None

    def visualize_callback(self, current_time, dt):
        if self.last_time is None:
            self.last_time = current_time

        if current_time - self.last_time < 1 / self.freq:
            return
        self.last_time = current_time
        state = self.state_subscriber.getLatestRbdStateViz()
        position = state[3:6]
        orientation = state[0:3]
        joint_positions = state[12:24]

        joint_positions = {
            "FL_hip_joint": joint_positions[0],
            "FL_thigh_joint": joint_positions[1],
            "FL_calf_joint": joint_positions[2],
            "RL_hip_joint": joint_positions[3],
            "RL_thigh_joint": joint_positions[4],
            "RL_calf_joint": joint_positions[5],
            "FR_hip_joint": joint_positions[6],
            "FR_thigh_joint": joint_positions[7],
            "FR_calf_joint": joint_positions[8],
            "RR_hip_joint": joint_positions[9],
            "RR_thigh_joint": joint_positions[10],
            "RR_calf_joint": joint_positions[11],
        }

        # Turn orientation
        orientation = ocs2rpy2quat(orientation)

        # Log the new state
        self.robot_logger.log_state(
            logtime=current_time, base_position=position, base_orientation=orientation, joint_positions=joint_positions
        )


class PyBulletCommandPublisher(CommandPublisher):
    def __init__(self, robot_id):
        super().__init__()
        self.robot_id = robot_id
        self.last_time = time.time()
        self.publish_count = 0
        self.start_time = time.time()

    def publish(self, commands):
        global kp
        global kd
        global desired_joint_angles
        kp = commands[0].kp
        kd = commands[0].kd
        for command in commands:
            desired_joint_angles[joint2idx[command.joint_name]] = command.desired_position

def physics_fn(robot_id, dt, lock, subscriber):
    global desired_joint_angles
    global running
    
    while running:
        with lock:
            for joint_name, joint_idx in joint2idx.items():
                joint_state = p.getJointState(robot_id, joint_name_to_id[joint_name])
                current_pos = joint_state[0]
                current_vel = joint_state[1]
                
                desired_pos = desired_joint_angles[joint_idx]
                desired_vel = 0
                
                torque = pd_controller(current_pos, desired_pos, desired_vel, current_vel, kp, kd)
                p.setJointMotorControl2(
                    robot_id,
                    joint_name_to_id[joint_name], 
                    p.VELOCITY_CONTROL,
                    targetVelocity=0,
                    force=0
                )
                p.setJointMotorControl2(
                    robot_id,
                    joint_name_to_id[joint_name],
                    p.TORQUE_CONTROL,
                    force=torque
                )
            p.stepSimulation()
            subscriber.needs_update = True
            time.sleep(dt)

class DummyChangeControllerSubscriber(ChangeControllerSubscriber):
    def __init__(self):
        super().__init__()
        self._callback = None
        self.new_controller = None
        self.new_controller = "SIT"

    def setCallbackFunction(self, callback):
        self._callback = callback

    def triggerCallbacks(self):
        if self._callback is not None and self.new_controller is not None:
            self._callback(self.new_controller)
            self.new_controller = None

    def stand_callback(self):
        self.new_controller = "STAND"

    def sit_callback(self):
        self.new_controller = "SIT"

    def bob_callback(self):
        self.new_controller = "WTW"


class DummyReferenceVelocityGenerator(ReferenceVelocityGenerator):
    def __init__(self, ui_controller: UIController):
        super().__init__()
        self.ui_controller = ui_controller

    def getReferenceVelocity(self, time, dt):
        ref = ReferenceVelocity()
        ref.velocity_x = self.ui_controller.linear_x
        ref.velocity_y = self.ui_controller.linear_y
        ref.yaw_rate = self.ui_controller.angular_z
        return ref

# Set up PyBullet
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load robot and ground
robot = p.loadURDF("./ocs2_robotic_assets/resources/go2/urdf/go2_description.urdf", [0, 0, 0.5])
ground = p.loadURDF("plane.urdf")

# Get joint IDs
joint_name_to_id = {}
for j in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, j)
    joint_id = info[0]
    joint_name = info[1].decode('UTF-8')
    joint_type = info[2]
    
    if joint_type == p.JOINT_REVOLUTE:
        print(f"Adding joint {joint_name} with id {joint_id}")
        joint_name_to_id[joint_name] = joint_id

# Set simulation parameters
dt = 0.001
p.setTimeStep(dt)
p.setGravity(0, 0, -9.81)
p.setPhysicsEngineParameter(
    fixedTimeStep=dt,
    numSolverIterations=100,
    numSubSteps=1
)

# Initialize joint positions
sit_joint_angles = np.array([0.0, 0.806, -1.802, 0.0, 0.806, -1.802, 0.0, 0.996, -1.802, 0.0, 0.996, -1.802])
stand_joint_angles = np.array([0.0, 0.806, -1.802, 0.0, 0.806, -1.802, 0.0, 0.996, -1.802, 0.0, 0.996, -1.802])

for joint_name in joint2idx.keys():
    p.resetJointState(robot, joint_name_to_id[joint_name], sit_joint_angles[joint2idx[joint_name]])
    p.setJointMotorControl2(robot, joint_name_to_id[joint_name], p.VELOCITY_CONTROL, targetVelocity=0, force=0)

desired_joint_angles = stand_joint_angles.copy()

lock = threading.Lock()

# Initialize tbai components
subscriber = PyBulletStateSubscriber(robot)
publisher = PyBulletCommandPublisher(robot)
controller_sub = DummyChangeControllerSubscriber()

rerun_logger = RerunLoggerNode(subscriber)

ui_controller = UIController(
    stand_callback=controller_sub.stand_callback,
    sit_callback=controller_sub.sit_callback,
    bob_callback=controller_sub.bob_callback,
)
ref_vel_gen = DummyReferenceVelocityGenerator(ui_controller)

tbai_python.write_init_time()

central_controller = tbai_python.CentralController.create(subscriber, publisher, controller_sub)
central_controller.add_bob_controller(subscriber, ref_vel_gen, rerun_logger.visualize_callback)
central_controller.add_static_controller(subscriber, rerun_logger.visualize_callback)
central_controller.add_wtw_controller(subscriber, ref_vel_gen, rerun_logger.visualize_callback)

# Start physics thread
physics_thread = threading.Thread(target=physics_fn, args=(robot, dt, lock, subscriber))
physics_thread.start()

central_controller.startThread()

try:
    ui_controller.run()
except KeyboardInterrupt:
    running = False
    rerun_store("go2.rrd")
    print("Stopping threads")
    central_controller.stopThread()
    physics_thread.join()
    p.disconnect()
