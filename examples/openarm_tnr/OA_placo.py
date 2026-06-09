import numpy as np
import placo

class OpenArmPlaco:
    def __init__(self, urdf_path, collision_path, config, posture_target, velocity_limit):
        # Init robot model from urdf
        self.robot = placo.RobotWrapper(urdf_path)
        # Add specialized collision
        print("INFO: Collision pairs override")
        self.robot.load_collision_pairs(collision_path)
        # Init QP solver
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)
        self.dt = config["dt"]
        self.solver.dt = self.dt
        self.initial_pose = np.eye(4)

        # Mask gripper prismatic joints from computation
        print("INFO: Masking gripper prismatic joints from computation")
        removed_joints = [
            "openarm_left_finger_joint1", "openarm_left_finger_joint2",
            "openarm_right_finger_joint1", "openarm_right_finger_joint2"
        ]
        for i in range(len(removed_joints)):
            self.solver.mask_dof(removed_joints[i])

        # Init left end-effector Task (relative to the torso frame)
        self.left_ee_task = self.solver.add_relative_frame_task("openarm_torso_center_link", "openarm_left_hand_tcp", self.initial_pose)
        self.left_ee_task.configure("left_ee", "soft", config["cost_ee"][0], config["cost_ee"][3])
        # Init right end-effector Task (relative to the torso frame)
        self.right_ee_task = self.solver.add_relative_frame_task("openarm_torso_center_link", "openarm_right_hand_tcp", self.initial_pose)
        self.right_ee_task.configure("right_ee", "soft", config["cost_ee"][0], config["cost_ee"][3])
        # Init Posture task
        self.posture_task = self.solver.add_joints_task()
        self.posture_target = posture_target
        if posture_target is None:
            self.posture_task.set_joints({joint: 0.0 for joint in self.robot.joint_names()})
        else:
            self.posture_task.set_joints(self.posture_target)
        self.posture_task.configure("posture", "soft", config["cost_posture"])
        # Init lambda regularization
        self.solver.add_regularization_task(config["lambda_reg"])
        # Init joint limit constraint
        self.solver.enable_joint_limits(True)
        # Init velocity constraint
        if velocity_limit is not None:
            for joint_name, value in velocity_limit.items():
                self.robot.set_velocity_limit(joint_name, value)
        self.solver.enable_velocity_limits(True)
        # Init self-collision constraint
        self.collisions_constraint = self.solver.add_avoid_self_collisions_constraint()
        self.collisions_constraint.configure("avoid_self_collisions", "hard")
        self.collisions_constraint.self_collisions_margin = config["collision_margin"]
        self.collisions_constraint.self_collisions_trigger = config["collision_limit"]

    def get_ee_pose(self, q):
        self.robot.state.q = q
        self.robot.update_kinematics()
        T_world_torso = self.robot.get_T_world_frame("openarm_torso_center_link")
        T_world_L = self.robot.get_T_world_frame("openarm_left_hand_tcp")
        T_world_R = self.robot.get_T_world_frame("openarm_right_hand_tcp")
        T_torso_L = np.linalg.inv(T_world_torso) @ T_world_L
        T_torso_R = np.linalg.inv(T_world_torso) @ T_world_R
        return T_torso_L, T_torso_R

    def IK_solve(self, current_q, left_target_M, right_target_M, debug):
        self.robot.state.q = current_q
        self.left_ee_task.T_a_b = left_target_M
        self.right_ee_task.T_a_b = right_target_M
        self.solver.solve(True)
        self.robot.update_kinematics()
        if debug:
            self.solver.dump_status()
        return self.robot.state.q

    def get_gravity_torques(self, q):
        self.robot.state.q = q
        self.robot.update_kinematics()
        return self.robot.static_gravity_compensation_torques("openarm_torso_center_link")

    def check_feasibility(self, q_init, left_target_M, right_target_M, max_iterations=100, tolerance=1e-3):
        self.robot.state.q = q_init
        self.robot.update_kinematics()
        q_backup = q_init.copy()
        for i in range(max_iterations):
            self.left_ee_task.T_a_b = left_target_M
            self.right_ee_task.T_a_b = right_target_M
            self.solver.solve(True)
            T_L_cur, T_R_cur = self.get_ee_pose(self.robot.state.q)
            err_l = np.linalg.norm(T_L_cur[:3, 3] - left_target_M[:3, 3])
            err_r = np.linalg.norm(T_R_cur[:3, 3] - right_target_M[:3, 3])
            if err_l < tolerance and err_r < tolerance:
                self.robot.state.q = q_backup
                self.robot.update_kinematics()
                return True, (err_l + err_r)
        T_L_cur, T_R_cur = self.get_ee_pose(self.robot.state.q)
        final_error = (np.linalg.norm(T_L_cur[:3, 3] - left_target_M[:3, 3]) +
                       np.linalg.norm(T_R_cur[:3, 3] - right_target_M[:3, 3]))
        self.robot.state.q = q_backup
        self.robot.update_kinematics()
        return False, final_error

    def check_traj_reachability(self, q_init, trajectory, tolerance=0.02, max_iter=50):
        self.robot.state.q = q_init.copy()
        self.robot.update_kinematics()
        for i, point in enumerate(trajectory):
            target_L = np.array(point["TL"])
            target_R = np.array(point["TR"])
            success = False
            for _ in range(max_iter):
                self.left_ee_task.T_a_b  = target_L
                self.right_ee_task.T_a_b = target_R
                self.solver.solve(True)
                self.robot.update_kinematics()
                T_L_cur, T_R_cur = self.get_ee_pose(self.robot.state.q)
                err_l = np.linalg.norm(T_L_cur[:3, 3] - target_L[:3, 3])
                err_r = np.linalg.norm(T_R_cur[:3, 3] - target_R[:3, 3])
                if err_l < tolerance and err_r < tolerance:
                    success = True
                    break
            if not success:
                print(f"Waypoint {i} unreachable (err_l={err_l:.4f} m  err_r={err_r:.4f} m)")
                return False
        return True
