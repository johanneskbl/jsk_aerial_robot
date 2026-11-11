import os, sys
import numpy as np
import casadi as ca
from abc import ABC, abstractmethod
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import yaml
import rospkg


class RecedingHorizonBase(ABC):
    """
    Base class for the receding horizon methods NMPC and MHE.

    :param string method: Sets correct path to save c generated code and is either "nmpc" or "wrench_est".
    :param bool build: Flag to build a solver as c generated code. Default: True
    """

    # Frames the cost function can track, set by "tracking_frame" in the config file.
    # NOTE: The C++ controller reads the same key from the same file with the same default.
    TRACKING_FRAMES = ("cog_tracking", "ee_tracking")
    DEFAULT_TRACKING_FRAME = "cog_tracking"

    def __init__(self, method, build: bool = True):
        self._acados_model = self.create_acados_model()
        self._create_acados_ocp()
        self._mkdir(method, self._acados_model.name)
        self._ocp_solver = self.create_acados_ocp_solver(build)

    def read_params(self, mode, method, robot_package, file_name):
        # Read parameters from configuration file in the robot's package
        # 'mode' is either "control" or "estimation"
        # 'method' is either "nmpc" or "mhe"
        rospack = rospkg.RosPack()
        try:
            param_path = os.path.join(rospack.get_path(robot_package), "config", file_name)
        except rospkg.common.ResourceNotFound:  # non-ROS environment
            # Fallback: construct absolute path from current file
            this_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.abspath(os.path.join(this_dir, "../../../.."))
            param_path = os.path.join(project_root, "robots", robot_package, "config", file_name)

        try:
            with open(param_path, "r") as f:
                param_dict = yaml.load(f, Loader=yaml.FullLoader)
            self.params = param_dict[mode][method]
        except FileNotFoundError:
            raise FileNotFoundError(f"Configuration file {param_path} not found.")
        except KeyError:
            raise KeyError(f"Mode or method not found in configuration file {param_path}.")

        # Compute number of shooting intervals or "steps" (or "nodes") along the horizon
        self.params["N_steps"] = int(self.params["T_horizon"] / self.params["T_step"])

        # Frame tracked by the cost function
        self.tracking_frame = self.params.get("tracking_frame", self.DEFAULT_TRACKING_FRAME)
        if self.tracking_frame not in self.TRACKING_FRAMES:
            raise ValueError(
                f"Invalid tracking_frame '{self.tracking_frame}' in {param_path}. Choose from {self.TRACKING_FRAMES}."
            )

    def _create_acados_ocp(self) -> AcadosOcp:
        # Gets called from child class's implementation
        # Create OCP object and set basic properties
        self._ocp = AcadosOcp()
        acados_source_path = os.environ["ACADOS_SOURCE_DIR"]  # Get acados source path from environment variable
        self._ocp.acados_include_path = acados_source_path + "/include"
        self._ocp.acados_lib_path = acados_source_path + "/lib"
        self._ocp.model = self._acados_model

        # TODO I think not needed but not sure
        sys.path.insert(0, acados_source_path)

        # Set parameters
        self._ocp.solver_options.N_horizon = self.params["N_steps"]  # Number of time steps along the prediction horizon
        self._ocp.dims.np = self._acados_model.p.size()[0]  # Number of parameters
        self._ocp.parameter_values = np.zeros(self._ocp.dims.np)  # Initialize parameters with zeros

    def get_acados_model(self) -> AcadosModel:
        return self._acados_model

    def get_ocp(self) -> AcadosOcp:
        return self._ocp

    def get_ocp_solver(self) -> AcadosOcpSolver:
        return self._ocp_solver

    @abstractmethod
    def create_acados_model(self) -> AcadosModel:
        pass

    @abstractmethod
    def create_acados_ocp_solver(self, build: bool = True) -> AcadosOcpSolver:
        pass

    @staticmethod
    def _mkdir(method, model_name):
        # Make a directory for generating cpp files of the acados model and solver
        # 'method' is either "nmpc" or "wrench_est"
        rospack = rospkg.RosPack()
        try:
            folder_path = os.path.join(
                rospack.get_path("aerial_robot_control"), "include", "aerial_robot_control", method, model_name
            )
        except rospkg.common.ResourceNotFound:  # non-ROS environment
            # Fallback: construct absolute path from current file
            this_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.abspath(os.path.join(this_dir, "../../../.."))
            folder_path = os.path.join(
                project_root, "aerial_robot_control", "include", "aerial_robot_control", method, model_name
            )

        os.makedirs(folder_path, exist_ok=True)
        os.chdir(folder_path)  # Change working directory to the model folder to save model in cwd

    # --------- Utility functions ---------
    def transform_cost_state(self):
        """
        Transform the CoG state to the frame tracked by the cost function

        :return p_t: Position of the tracked frame in World frame
        :return v_t: Linear velocity of the tracked frame in World frame
        :return qe_w, qe_x, qe_y, qe_z: Quaternion error qe = q_ref^* * q_t
        :return w_t: Angular velocity of the tracked frame, expressed in the tracked frame
        """
        # Get the CoG state from the model
        p_t, v_t, w_t = self.p, self.v, self.w
        q_t = (self.qw, self.qx, self.qy, self.qz)

        if self.tracking_frame == "ee_tracking":
            ee_qw, ee_qx, ee_qy, ee_qz = self.ee_q[0], self.ee_q[1], self.ee_q[2], self.ee_q[3]
            rot_wb = self._get_rot_wb_ca(self.qw, self.qx, self.qy, self.qz)
            rot_be = self._get_rot_wb_ca(ee_qw, ee_qx, ee_qy, ee_qz)
            rot_eb = rot_be.T

            p_t = self.p + rot_wb @ self.ee_p
            v_t = self.v + rot_wb @ ca.cross(self.w, self.ee_p)
            w_t = rot_eb @ self.w
            q_t = self._quaternion_multiply(self.qw, self.qx, self.qy, self.qz, ee_qw, ee_qx, ee_qy, ee_qz)

        qe_w, qe_x, qe_y, qe_z = self._quaternion_multiply(self.qwr, -self.qxr, -self.qyr, -self.qzr, *q_t)

        return p_t, v_t, qe_w, qe_x, qe_y, qe_z, w_t

    def transform_cost_impedance(self, lin_acc_w, ang_acc_b, f_ext_w, tau_ext_b):
        """
        Transform the terms of the impedance cost from the CoG to the frame tracked by the cost function

        :param lin_acc_w: Linear acceleration of the CoG in World frame
        :param ang_acc_b: Angular acceleration in Body frame
        :param f_ext_w: External force in World frame
        :param tau_ext_b: External torque about the CoG in Body frame
        :return lin_acc_t: Linear acceleration of the Tracked frame in World frame
        :return ang_acc_t: Angular acceleration, expressed in Tracked frame
        :return f_ext_w: External force in World frame (the same for every point of the body)
        :return tau_ext_t: External torque about the origin of Tracked frame, expressed in Tracked frame
        """
        if self.tracking_frame == "cog_tracking":
            return lin_acc_w, ang_acc_b, f_ext_w, tau_ext_b

        rot_wb = self._get_rot_wb_ca(self.qw, self.qx, self.qy, self.qz)
        rot_be = self._get_rot_wb_ca(self.ee_q[0], self.ee_q[1], self.ee_q[2], self.ee_q[3])
        rot_eb = rot_be.T

        # Acceleration of a point fixed on the body: a_t = a_b + R_wb @ (alpha_b x ee_p + w_b x (w_b x ee_p))
        lin_acc_t = lin_acc_w + rot_wb @ (
            ca.cross(ang_acc_b, self.ee_p) + ca.cross(self.w, ca.cross(self.w, self.ee_p))
        )
        # The tracked frame shares the angular acceleration of the body
        ang_acc_t = rot_eb @ ang_acc_b
        # Torque about the origin of the tracked frame: tau_t = tau_b - ee_p x f_b
        f_ext_b = rot_wb.T @ f_ext_w
        tau_ext_t = rot_eb @ (tau_ext_b - ca.cross(self.ee_p, f_ext_b))

        return lin_acc_t, ang_acc_t, f_ext_w, tau_ext_t

    @staticmethod
    def _get_rot_around_x(angle):
        """
        :return: Rotation matrix for a rotation around the X axis by the given angle
        """
        return ca.vertcat(
            ca.horzcat(1, 0, 0),
            ca.horzcat(0, ca.cos(angle), -ca.sin(angle)),
            ca.horzcat(0, ca.sin(angle), ca.cos(angle)),
        )

    @staticmethod
    def _get_rot_wb_ca(qw, qx, qy, qz):
        row_1 = ca.horzcat(
            ca.MX(1 - 2 * qy ** 2 - 2 * qz ** 2),
            ca.MX(2 * qx * qy - 2 * qw * qz),
            ca.MX(2 * qx * qz + 2 * qw * qy)
        )
        row_2 = ca.horzcat(
            ca.MX(2 * qx * qy + 2 * qw * qz),
            ca.MX(1 - 2 * qx ** 2 - 2 * qz ** 2),
            ca.MX(2 * qy * qz - 2 * qw * qx)
        )
        row_3 = ca.horzcat(
            ca.MX(2 * qx * qz - 2 * qw * qy),
            ca.MX(2 * qy * qz + 2 * qw * qx),
            ca.MX(1 - 2 * qx ** 2 - 2 * qy ** 2)
        )
        rot_wb = ca.vertcat(row_1, row_2, row_3)
        return rot_wb

    @staticmethod
    def _get_rot_wb_np(qw, qx, qy, qz):
        """
        Create a rotation matrix from quaternions
        :param qw: Quaternion component w
        :param qx: Quaternion component x
        :param qy: Quaternion component y
        :param qz: Quaternion component z
        :return: Rotation matrix from World to Body frame
        """
        return np.array(
            [
                [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
                [2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)],
                [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)],
            ]
        )

    @staticmethod
    def _get_skew_symmetric_matrix(vector):
        """
        Create a skew-symmetric matrix from a 3D vector
        :param vector: 3D vector to create the skew-symmetric matrix from
        :return: Skew-symmetric matrix
        """
        skew_matrix = ca.vertcat(
            ca.horzcat(0, -vector[2], vector[1]),
            ca.horzcat(vector[2], 0, -vector[0]),
            ca.horzcat(-vector[1], vector[0], 0),
        )

        return skew_matrix

    @staticmethod
    def _quaternion_multiply(qw1, qx1, qy1, qz1, qw2, qx2, qy2, qz2):
        """
        :return: Resulting quaternion in (w, x, y, z) format
        """
        return (
            qw1 * qw2 - qx1 * qx2 - qy1 * qy2 - qz1 * qz2,
            qw1 * qx2 + qx1 * qw2 + qy1 * qz2 - qz1 * qy2,
            qw1 * qy2 - qx1 * qz2 + qy1 * qw2 + qz1 * qx2,
            qw1 * qz2 + qx1 * qy2 - qy1 * qx2 + qz1 * qw2,
        )
