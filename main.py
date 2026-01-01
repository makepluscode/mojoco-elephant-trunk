import mujoco
import mujoco.viewer
import numpy as np
import time
import os

class ElephantTrunkController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.state = "IK"  # Default to IK mode
        
        # Virtual Joint Angles: [prox_pitch, prox_yaw, dist_pitch, dist_yaw] (Radians)
        self.q_virtual = np.zeros(4)
        self.limit = 1.047  # Approx 60 degrees
        
        # Physical Parameters
        self.coupling_coeff = 0.85
        self.ctrl_scale = 1.0
        self.offsets = np.zeros(8)
        
        # IK Parameters
        self.target_v = np.zeros(3)  # [vx, vy, vz]
        self.damping = 1e-4
        self.dt = model.opt.timestep
        
        # Cache IDs
        self.tip_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "d_tip1")
        self.last_log_time = 0.0

    def step(self):
        if self.state == "IK":
            self._apply_jacobian_ik()
        
        # Map Virtual Angles to Motors
        self._sync_motors()

    def _apply_jacobian_ik(self):
        # 1. Get Full Jacobian (3 x nv)
        # nv is 20 for this model (10 joints * 2 axes)
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.tip_site_id)
        
        # 2. Reduce Jacobian to Controllable 4 DoF
        # Our 4 virtual DoF are mapped as:
        # q_v[0, 1] -> Proximal Pitch/Yaw (Joints 1-5)
        # q_v[2, 3] -> Distal Pitch/Yaw (Joints 6-10)
        # Note: MuJoCo joints here are 2-DoF universal joints.
        # Indices for velocity (nv):
        # Proximal: 0-9 (5 joints * 2 DoF)
        # Distal: 10-19 (5 joints * 2 DoF)
        
        J_reduced = np.zeros((3, 4))
        
        # Sum columns for proximal joints (0-9)
        # Even indices (0, 2, 4, 6, 8) = Yaw, Odd (1, 3, 5, 7, 9) = Pitch
        J_reduced[:, 0] = np.sum(jacp[:, 1:10:2], axis=1) # Prox Pitch
        J_reduced[:, 1] = np.sum(jacp[:, 0:10:2], axis=1) # Prox Yaw
        
        # Sum columns for distal joints (10-19)
        J_reduced[:, 2] = np.sum(jacp[:, 11:20:2], axis=1) # Dist Pitch
        J_reduced[:, 3] = np.sum(jacp[:, 10:20:2], axis=1) # Dist Yaw
        
        # 3. Damped Pseudo-Inverse Solver
        # J_inv = J.T @ inv(J @ J.T + lambda^2 * I)
        JT = J_reduced.T
        JJT = J_reduced @ JT
        inv_part = np.linalg.inv(JJT + self.damping * np.eye(3))
        J_pseudo = JT @ inv_part
        
        # 4. Integrate Joint Velocity
        q_dot = J_pseudo @ self.target_v
        self.q_virtual += q_dot * self.dt * 10.0 # Gain multiplier for responsiveness
        
        # 5. Apply Limits
        self.q_virtual = np.clip(self.q_virtual, -self.limit, self.limit)

    def _sync_motors(self):
        p_pitch, p_yaw = self.q_virtual[0], self.q_virtual[1]
        d_pitch, d_yaw = self.q_virtual[2], self.q_virtual[3]

        # Proximal
        self.data.ctrl[2] = p_pitch * self.ctrl_scale + self.offsets[2]
        self.data.ctrl[3] = -p_pitch * self.ctrl_scale + self.offsets[3]
        self.data.ctrl[0] = p_yaw * self.ctrl_scale + self.offsets[0]
        self.data.ctrl[1] = -p_yaw * self.ctrl_scale + self.offsets[1]

        # Distal with Coupling
        self.data.ctrl[6] = (d_pitch * self.ctrl_scale) + (self.coupling_coeff * self.data.ctrl[2]) + self.offsets[6]
        self.data.ctrl[7] = (-d_pitch * self.ctrl_scale) + (self.coupling_coeff * self.data.ctrl[3]) + self.offsets[7]
        self.data.ctrl[4] = (d_yaw * self.ctrl_scale) + (self.coupling_coeff * self.data.ctrl[0]) + self.offsets[4]
        self.data.ctrl[5] = (-d_yaw * self.ctrl_scale) + (self.coupling_coeff * self.data.ctrl[1]) + self.offsets[5]

    def set_velocity(self, axis, val):
        self.target_v[axis] = val

    def reset(self):
        self.q_virtual.fill(0)
        self.target_v.fill(0)
        self.data.ctrl.fill(0)
        print("[INFO] Pose Reset to Zero.")

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
        v = 0.2
        
        # Cartesian XYZ Control (WASD + QE)
        if char == "W": controller.set_velocity(0, v)     # +X
        elif char == "S": controller.set_velocity(0, -v)  # -X
        elif char == "A": controller.set_velocity(1, v)     # +Y
        elif char == "D": controller.set_velocity(1, -v)  # -Y
        elif char == "Q": controller.set_velocity(2, v)     # +Z
        elif char == "E": controller.set_velocity(2, -v)  # -Z
        
        # System
        elif keycode == 32: # Space
            controller.target_v.fill(0)
            print("[STATUS] Velocity Cleared")
            print(f"[STATUS] Tip POS: {data.site_xpos[controller.tip_site_id].round(3)}")
            print(f"[STATUS] Target Angles: {controller.q_virtual.round(3)}")
        elif char == "R": controller.reset()

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        print("Project Elephant Trunk - Jacobian IK Mode Active")
        print("Cartesian Velocity Controls (WASD/QE):")
        print("  X-Axis (Fwd/Bwd): [W / S]")
        print("  Y-Axis (Left/Rt): [A / D]")
        print("  Z-Axis (Up/Down): [Q / E]")
        print("  System: [Space] Stop/Status, [R] Reset")

        # Disable default MuJoCo shortcuts that interfere with WASD/QE
        # We do this by forcing the visibility flags in the loop
        
        while viewer.is_running():
            step_start = time.time()
            
            # Force UI states to ignore default shortcuts (W, S, A, etc.)
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_WIREFRAME] = 0
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_SHADOW] = 1 # Keep shadows on
            
            controller.step()
            mujoco.mj_step(model, data)
            
            # Periodic logging
            if data.time - controller.last_log_time > 1.0:
                tip_pos = data.site_xpos[controller.tip_site_id]
                print(f"Time: {data.time:.1f} | Tip POS: {tip_pos.round(3)} | Target V: {controller.target_v}")
                controller.last_log_time = data.time

            viewer.sync()
            
            elapsed = time.time() - step_start
            if elapsed < model.opt.timestep:
                time.sleep(model.opt.timestep - elapsed)

if __name__ == "__main__":
    main()
