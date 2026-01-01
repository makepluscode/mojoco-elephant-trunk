import mujoco
import mujoco.viewer
import numpy as np
import time
import os


class ElephantTrunkController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.state = "JOYSTICK"  # Default to Joystick mode

        # Target Angles: [prox_pitch, prox_yaw, dist_pitch, dist_yaw] (Radians)
        self.target_angles = np.zeros(4)
        self.step_size = 0.05
        self.limit = 1.047  # Approx 60 degrees

        # Physical Parameters
        self.coupling_coeff = 0.85
        self.ctrl_scale = 1.0  # Mapping angle to actuator ctrl range [-1, 1]
        self.offsets = np.zeros(8)

        # Cache IDs
        self.proximal_geom_ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"p{i}_geom")
            for i in range(1, 6)
        ]
        self.tip_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "d_tip1")

        self.last_log_time = 0.0

    def step(self):
        # Update Motor Commands from Target Angles
        self._apply_joystick_logic()

        # Update colors based on state if needed
        pass

    def _apply_joystick_logic(self):
        # 1. Proximal Commands (Indices: 0=Yaw, 1=Yaw, 2=Pitch, 3=Pitch)
        # Pitch: Target[0], Yaw: Target[1]
        p_pitch = self.target_angles[0]
        p_yaw = self.target_angles[1]

        # 2. Distal Commands (Indices: 4=Yaw, 5=Yaw, 6=Pitch, 7=Pitch)
        # Pitch: Target[2], Yaw: Target[3]
        d_pitch = self.target_angles[2]
        d_yaw = self.target_angles[3]

        # Calculate base motor strokes (Differential drive for each axis)
        # Proximal
        self.data.ctrl[2] = p_pitch * self.ctrl_scale + self.offsets[2]  # Pitch P
        self.data.ctrl[3] = -p_pitch * self.ctrl_scale + self.offsets[3]  # Pitch N
        self.data.ctrl[0] = p_yaw * self.ctrl_scale + self.offsets[0]  # Yaw P
        self.data.ctrl[1] = -p_yaw * self.ctrl_scale + self.offsets[1]  # Yaw N

        # Distal with Coupling Compensation
        # Distal_Stoke = Scale(theta_dist) + k_coupling * Proximal_Stroke
        self.data.ctrl[6] = (
            (d_pitch * self.ctrl_scale)
            + (self.coupling_coeff * self.data.ctrl[2])
            + self.offsets[6]
        )
        self.data.ctrl[7] = (
            (-d_pitch * self.ctrl_scale)
            + (self.coupling_coeff * self.data.ctrl[3])
            + self.offsets[7]
        )
        self.data.ctrl[4] = (
            (d_yaw * self.ctrl_scale)
            + (self.coupling_coeff * self.data.ctrl[0])
            + self.offsets[4]
        )
        self.data.ctrl[5] = (
            (-d_yaw * self.ctrl_scale)
            + (self.coupling_coeff * self.data.ctrl[1])
            + self.offsets[5]
        )

    def set_angle(self, idx, direction):
        self.target_angles[idx] += direction * self.step_size
        self.target_angles[idx] = np.clip(
            self.target_angles[idx], -self.limit, self.limit
        )

    def reset(self):
        self.target_angles.fill(0)
        self.data.ctrl.fill(0)
        print("[INFO] Target Angles Reset to Zero.")


def main():
    model_path = "elephant_trunk.xml"
    if not os.path.exists(model_path):
        print(f"Error: {model_path} not found.")
        return

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    controller = ElephantTrunkController(model, data)

    def key_callback(keycode):
        char = chr(keycode).upper() if 32 <= keycode <= 126 else None
        
        # Letters (Handle both cases)
        if char == "I":
            print("[CMD] Prox Pitch Up")
            controller.set_angle(0, -1)
        elif char == "K":
            print("[CMD] Prox Pitch Down")
            controller.set_angle(0, 1)
        elif char == "J":
            print("[CMD] Prox Yaw Left")
            controller.set_angle(1, -1)
        elif char == "L":
            print("[CMD] Prox Yaw Right")
            controller.set_angle(1, 1)
        
        # Arrow Keys
        elif keycode == 265: # Up
            print("[CMD] Dist Pitch Up")
            controller.set_angle(2, -1)
        elif keycode == 264: # Down
            print("[CMD] Dist Pitch Down")
            controller.set_angle(2, 1)
        elif keycode == 263: # Left
            print("[CMD] Dist Yaw Left")
            controller.set_angle(3, -1)
        elif keycode == 262: # Right
            print("[CMD] Dist Yaw Right")
            controller.set_angle(3, 1)
        
        elif char == "R":
            controller.reset()
        elif keycode == 32: # Space
            print(f"[STATUS] Target Angles (rad): {controller.target_angles.round(3)}")
            print(f"[STATUS] Control Values: {controller.data.ctrl.round(1)}")

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        print("Project Elephant Trunk - Virtual Joystick Active")
        print("Controls (Avoids MuJoCo conflicts):")
        print("  Proximal: [I/K] Pitch, [J/L] Yaw")
        print("  Distal:   [Up/Dn] Pitch, [Lt/Rt] Yaw")
        print("  System:   [R] Reset, [Space] Status")

        while viewer.is_running():
            step_start = time.time()

            # Update targets and step physics
            controller.step()
            mujoco.mj_step(model, data)

            # Periodic logging
            if data.time - controller.last_log_time > 1.0:
                print(
                    f"Time: {data.time:.1f} | Prox [P:{controller.target_angles[0]:.2f} Y:{controller.target_angles[1]:.2f}] | Dist [P:{controller.target_angles[2]:.2f} Y:{controller.target_angles[3]:.2f}]"
                )
                controller.last_log_time = data.time

            viewer.sync()

            # Sync to real-time
            elapsed = time.time() - step_start
            if elapsed < model.opt.timestep:
                time.sleep(model.opt.timestep - elapsed)


if __name__ == "__main__":
    main()
