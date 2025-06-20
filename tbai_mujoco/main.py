import mujoco
import numpy as np
import time
import threading

from mujoco import viewer


def pd_controller(
    q_current: np.ndarray,
    q_desired: np.ndarray,
    v_desired: np.ndarray,
    v_current: np.ndarray,
    kp: float,
    kd: float,
) -> np.ndarray:
    return kp * (q_desired - q_current) + kd * (v_desired - v_current)


my_ctrl = np.zeros_like(12)


class Go2StaticController:
    def __init__(
        self,
        model: mujoco.MjModel,
        kp: float,
        kd: float,
        q_initial: np.ndarray,
        q_desired: np.ndarray,
        alpha_speed: float = 1.0,
    ):
        self.model = model
        self.kp = kp
        self.kd = kd
        self.q_initial = q_initial
        self.q_desired = q_desired
        self.alpha = 0.0
        self.alpha_speed = alpha_speed

        print(f"q initial: {q_initial}")
        print(f"q desired: {q_desired}")

    @staticmethod
    def update_alpha(alpha_current: float, alpha_speed: float, dt: float) -> float:
        assert alpha_speed > 0, "alpha_speed must be positive"
        return min(alpha_current + alpha_speed * dt, 1.0)

    @staticmethod
    def interpolate(from_: np.ndarray, to_: np.ndarray, alpha: float) -> np.ndarray:
        assert 0 <= alpha <= 1, "alpha must be between 0 and 1"
        return from_ * (1 - alpha) + to_ * alpha

    @staticmethod
    def get_joint_positions(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
        del model  # unused
        return data.qpos[-12:]

    @staticmethod
    def get_joint_velocities(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
        del model  # unused
        return data.qvel[-12:]

    def get_action(self, data: mujoco.MjData, dt: float) -> np.ndarray:
        self.alpha = self.update_alpha(self.alpha, self.alpha_speed, dt)
        q = self.get_joint_positions(self.model, data)
        v = self.get_joint_velocities(self.model, data)

        q_desired = self.interpolate(from_=self.q_initial, to_=self.q_desired, alpha=self.alpha)
        v_desired = np.zeros_like(v)

        return q_desired


def viewer_fn(window, lock, dt):
    while window.is_running():
        with lock:
            window.sync()
        time.sleep(dt)



def physics_fn(model, data, lock, dt):
    global my_ctrl
    while True:
        with lock:
            current_q = data.qpos[7 : 7 + 12]
            desired_q = my_ctrl.copy()
            current_v = data.qvel[6 : 6 + 12]
            desired_v = np.zeros_like(current_v)
            data.ctrl[:12] = pd_controller(current_q, desired_q, desired_v, current_v, 30, 0.5)
            mujoco.mj_step(model, data)
        time.sleep(dt)


def controller_fn(model, data, controller, lock, dt):
    global my_ctrl
    while True:
        with lock:
            ctrl = controller.get_action(data, dt)
            my_ctrl = ctrl.copy()
        time.sleep(dt)

scene_path = "/home/kuba/Documents/mujoco_menagerie/unitree_go2/scene.xml"
model = mujoco.MjModel.from_xml_path(scene_path)
model.opt.timestep = 0.001
data = mujoco.MjData(model)


print("Joint names:")
for i in range(1, 12 + 1):  # first joint is a virtual world->base_link joint
    print(f"  ({i})\t{mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)}")

sit_joint_angles = np.array([0.0, 1.806, -2.102, 0.0, 1.806, -2.102, 0.0, 1.996, -2.102, 0.0, 1.996, -2.102])
stand_joint_angles = np.array([0.0, 0.806, -1.802, 0.0, 0.806, -1.802, 0.0, 0.996, -1.802, 0.0, 0.996, -1.802])

data.qpos[-12:] = sit_joint_angles

static_controller = Go2StaticController(model, 30, 0.5, sit_joint_angles, stand_joint_angles, 1)

window = mujoco.viewer.launch_passive(model, data)
lock = threading.Lock()

viewer_thread = threading.Thread(target=viewer_fn, args=(window, lock, 1 / 30))
viewer_thread.start()

physics_thread = threading.Thread(target=physics_fn, args=(model, data, lock, model.opt.timestep))
physics_thread.start()

controller_thread = threading.Thread(target=controller_fn, args=(model, data, static_controller, lock, 1 / 50))
controller_thread.start()

viewer_thread.join()
physics_thread.join()
controller_thread.join()
print(model)
print(data)
