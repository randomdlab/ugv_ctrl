from nav_msgs.msg import Odometry
import casadi as ca
import numpy as np
from math import sin, cos
from scipy.io import loadmat, savemat
from controllers.utils import odom_to_yaw


# 中心对中心误差
def error_state(X_r, X_f) -> np.ndarray:
    x_r, y_r, theta_r = X_r.item(0), X_r.item(1), X_r.item(2)
    x_f, y_f, theta_f = X_f.item(0), X_f.item(1), X_f.item(2)
    mtx = np.array(
        [
            [cos(theta_f), sin(theta_f), 0.0],
            [-sin(theta_f), cos(theta_f), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    vec = np.array([[x_r - x_f], [y_r - y_f], [theta_r - theta_f]])
    return mtx @ vec


# 车头对车头误差
# def error_state(X_r: np.ndarray, X_f: np.ndarray, rho: float) -> np.ndarray:
#     def c2h(x):
#         theta = x.item(2)
#         mat = np.array(
#             [
#                 [1, rho * cos(theta)],
#                 [1, rho * sin(theta)],
#                 [0, 1]
#             ]
#         )
#         return mat @ x
#     X_r = c2h(X_r)
#     X_f = c2h(X_f)
#     x_r, y_r, theta_r = X_r.item(0), X_r.item(1), X_r.item(2)
#     x_f, y_f, theta_f = X_f.item(0), X_f.item(1), X_f.item(2)
#     mtx = np.array(
#         [
#             [cos(theta_f), sin(theta_f), 0.0],
#             [-sin(theta_f), cos(theta_f), 0.0],
#             [0.0, 0.0, 1.0],
#         ]
#     )
#     vec = np.array([[x_r - x_f], [y_r - y_f], [theta_r - theta_f]])
#     return mtx @ vec


def next_error_state(X: ca.MX, u_f: ca.MX, u_r: ca.MX, rho: float, dt: float) -> ca.MX:
    v_f, w_f = u_f[0], u_f[1]
    v_r, w_r = u_r[0], u_r[1]

    def f(X: ca.MX) -> ca.MX:
        x_e, y_e, theta_e = X[0, 0], X[1, 0], X[2, 0]
        return ca.vertcat(
            -rho * w_r * ca.sin(theta_e) - v_f + v_r * ca.cos(theta_e) + w_f * y_e,
            rho * w_r * ca.cos(theta_e) + v_r * ca.sin(theta_e) + w_f * (-rho - x_e),
            -w_f + w_r,
        )

    k1 = f(X)
    k2 = f(X + dt / 2 * k1)
    k3 = f(X + dt / 2 * k2)
    k4 = f(X + dt * k3)
    x_dot = (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
    return X + k1 * dt


class mpc_controller:
    def __init__(self, matfile: str):
        self.__filename: str = matfile
        self.__last_u: np.ndarray = np.zeros((2, 1))
        self.__load_data()
        self.__prob = ca.Opti()
        self.__u_f = self.__prob.parameter(2, 1)
        self.__u_r = self.__prob.parameter(2, 1)
        self.__x_e = self.__prob.parameter(3, 1)
        self.__x_es: list[ca.MX] = []
        self.__x_es_prv: list[np.ndarray] = []
        self.__us: list[ca.MX] = [self.__prob.variable(2, 1) for _ in range(self.N)]
        self.__us_prv: list[np.ndarray] = []
        self.__cost = self.__prob.variable()
        self.__prob.minimize(self.__cost)
        self.ipopt_cfg = {
            "max_iter": 100,
            # "tol": 1e-6,
            "acceptable_tol": 1e-8,
            "acceptable_obj_change_tol": 1e-6,
            "print_level": 0,
            "sb": "yes",
            "linear_solver": "mumps",
        }
        self.solver_cfg = {
            "ipopt": self.ipopt_cfg,
            "print_time": False,
        }
        self.__build_prob()

    def __load_data(self):
        data = loadmat(self.__filename)
        self.N = data["N"].item()
        self.Q = ca.MX(data["Q"])
        self.R = ca.MX(data["R"])
        self.P = ca.MX(data["P"])
        self.K = ca.MX(data["K"])
        self.Omega = data["Omega"].item()
        self.X_u = data["X_u"]
        self.X_l = data["X_l"]
        self.U_u = data["U_u"]
        self.U_l = data["U_l"]
        self.Delta_U = data["Delta_U"]
        self.sample_time = data["sample_time"].item()
        self.rho = data["rho"].item()

    def __next_x_e(self, x_e: ca.MX, u: ca.MX):
        v_f, w_f = u[0], u[1]
        v_r, w_r = self.__u_r[0], self.__u_r[1]

        def f(X: ca.MX) -> ca.MX:
            x_e, y_e, theta_e = X[0, 0], X[1, 0], X[2, 0]
            return ca.vertcat(
                -self.rho * w_r * ca.sin(theta_e)
                - v_f
                + v_r * ca.cos(theta_e)
                + w_f * y_e,
                self.rho * w_r * ca.cos(theta_e)
                + v_r * ca.sin(theta_e)
                + w_f * (-self.rho - x_e),
                -w_f + w_r,
            )

        k1 = f(x_e)
        k2 = f(x_e + self.sample_time / 2 * k1)
        k3 = f(x_e + self.sample_time / 2 * k2)
        k4 = f(x_e + self.sample_time * k3)
        x_dot = (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        return x_e + k1 * self.sample_time

    def __build_prob(self):
        # add cost
        self.__x_es = []
        x_e = self.__next_x_e(self.__x_e, self.__u_f)
        for i in range(self.N):
            ui = self.__us[i]
            x_e = self.__next_x_e(x_e, ui)
            self.__cost += ui.T @ self.R @ ui
            self.__cost += x_e.T @ self.Q @ x_e
            self.__x_es.append(x_e)
        self.__cost += self.__x_es[-1].T @ self.P @ self.__x_es[-1]

        # add constraints
        for i in range(self.N):
            self.__prob.subject_to(self.__us[i] >= self.U_l)
            self.__prob.subject_to(self.__us[i] <= self.U_u)
            self.__prob.subject_to(self.__x_es[i] >= self.X_l)
            self.__prob.subject_to(self.__x_es[i] <= self.X_u)
            if i > 0:
                self.__prob.subject_to(self.__us[i] - self.__us[i - 1] <= self.Delta_U)
                self.__prob.subject_to(self.__us[i] - self.__us[i - 1] >= -self.Delta_U)
        self.__prob.subject_to(
            self.__x_es[-1].T @ self.P @ self.__x_es[-1] <= self.Omega
        )
        self.__prob.subject_to(
            self.K @ self.__x_es[-1] - self.__us[-1] >= -self.Delta_U
        )
        self.__prob.subject_to(self.K @ self.__x_es[-1] - self.__us[-1] <= self.Delta_U)
        self.__prob.solver("ipopt", self.solver_cfg)

    def __update_data(self, x_follower, u_follower, x_request, u_request):
        self.__prob.set_value(self.__u_f, u_follower)
        self.__prob.set_value(self.__u_r, u_request)
        self.__prob.set_value(self.__x_e, error_state(x_request, x_follower))
        if len(self.__us_prv) > 0:
            (
                self.__prob.set_initial(self.__us[i], self.__us_prv[i])
                for i in range(self.N)
            )

    def __clean_cache(self):
        self.__x_es_prv = []
        self.__us_prv = []

    def rebuild(self):
        self.__build_prob()

    def solve(self, x_follower, u_follower, x_request, u_request) -> np.ndarray:
        self.__update_data(x_follower, u_follower, x_request, u_request)
        self.__clean_cache()
        ret = np.zeros((2, 1))
        try:
            sol = self.__prob.solve()
            ret = sol.value(self.__us[0])
            (self.__us_prv.append(sol.value(self.__us[i]) for i in range(self.N)))
            (self.__x_es_prv.append(sol.value(self.__x_es[i]) for i in range(self.N)))
        except:
            # print("solve failed")
            # print(f"cost: {prob.debug.value(cost)}")
            ret = self.__prob.debug.value(self.__us[0])
            (
                self.__us_prv.append(self.__prob.debug.value(self.__us[i]))
                for i in range(self.N)
            )
            (
                self.__x_es_prv.append(self.__prob.debug.value(self.__us[i]))
                for i in range(self.N)
            )
        ret = np.reshape(ret, (-1, 1))
        delta_u = np.clip(ret - self.__last_u, -self.Delta_U, self.Delta_U)
        ret = self.__last_u + delta_u
        self.__last_u = ret
        return ret


mat_file = "/workspaces/mpc_ws/src/ugv_ctrl/scripts/controllers/mpc.mat"
ros_ctrl = mpc_controller(mat_file)
# ros_ctrl.N = 10
ros_ctrl.sample_time = 0.1


def calc_cmd(targets: list[Odometry], pose: Odometry) -> tuple[float, float]:
    tgt = targets[0]
    yaw_r = odom_to_yaw(tgt)
    x_r = np.array([[tgt.pose.pose.position.x], [tgt.pose.pose.position.y], [yaw_r]])
    u_r = np.array([[tgt.twist.twist.linear.x], [tgt.twist.twist.angular.z]])
    yaw_f = odom_to_yaw(pose)
    x_f = np.array([[pose.pose.pose.position.x], [pose.pose.pose.position.y], [yaw_f]])
    u_f = np.array([[pose.twist.twist.linear.x], [pose.twist.twist.angular.z]])
    u = ros_ctrl.solve(x_f, u_f, x_r, u_r)
    return u.item(0), u.item(1)


if __name__ == "__main__":
    mat_file = "/workspaces/mpc_ws/mpc.mat"
    x_r = np.array([[0], [1.0], [0]])
    u_r = np.array([[0.8], [0.2]])
    x_f = np.array([[0], [0], [0]])
    u_f = np.array([[1.0], [0.2]])
    ctrl = mpc_controller(mat_file)

    u = ctrl.solve(x_f, u_f, x_r, u_r)
    print(u.T)
