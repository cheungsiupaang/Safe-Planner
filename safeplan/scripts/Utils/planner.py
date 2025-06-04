from casadi import MX, vertcat, nlpsol, veccat
from Utils.polynomial import Polynomial
import numpy as np
from numpy import cos, sin, sqrt, abs, pi
import yaml
import os
import rospy

current_dir = os.path.dirname(__file__)


class Planner:
    def __init__(self, dynconstraint, planner_param, dim, order, hypers=None, hypers_shrinked=None, waypoints=None):
        self.dim_c = dim['c']
        self.dim_a = dim['a']
        self.dim_b = dim['b']
        self.order_c = order['c']
        self.order_a = order['a']
        self.order_b = order['b']
        self.dyncons = dynconstraint
        self.traj_num = len(hypers)
        self.coeff, self.coeff_g = [], []
        self.g, self.lb, self.ub = [], [], []
        self.J = 0
        self.T_g = 3
        self.m_steps = 15
        ipopt_options = {'max_iter': 10000, 'acceptable_tol': 1e-4, 'acceptable_obj_change_tol': 1e-5,
                         'tol': 1e-4, 'mumps_mem_percent': 50000}
        self.solver_options = {'ipopt': ipopt_options}
        self.nlp = {}
        self.x_sol, self.g_sol = None, None
        self.T_lambda = planner_param['T_lambda']
        self.d4x_lambda = planner_param['d4x_lambda'] 
        self.ddpsi_lambda = planner_param['ddpsi_lambda']
        self.ddtheta_lambda = planner_param['ddtheta_lambda']
        self.xef_lambda = planner_param['xef_lambda']
        self.hypers = hypers
        self.hypers_shrinked = hypers_shrinked
        self.waypoints = waypoints
        self.avoid_method = planner_param['avoid_method']

    def load_param(self, filename):
        print('Loading planner parameters from' + filename)
        with open(filename, 'r') as file:
            _param = yaml.load(file, Loader=yaml.FullLoader)
        self.T_lambda = _param['T_lambda']
        self.d4x_lambda = _param['d4x_lambda']
        self.ddpsi_lambda = _param['ddpsi_lambda']
        self.ddtheta_lambda = _param['ddtheta_lambda']
        self.xef_lambda = _param['xef_lambda']

    def setup(self):
        _x = []
        for i in range(len(self.waypoints)):
            if i != len(self.waypoints) - 1:
                _x.append(self.waypoints[i])
            else:
                _xef = self.waypoints[i]
        _Ref = self.dyncons.Ref
        _v0 = self.dyncons.v0
        _a0 = self.dyncons.a0
        _j0 = self.dyncons.j0
        _psi0 = self.dyncons.psi0
        _theta0 = self.dyncons.theta0
        _dpsi0 = self.dyncons.dpsi0
        _dtheta0 = self.dyncons.dtheta0
        (vT_, aT_, jT_, psiT_, thetaT_, dpsiT_, dthetaT_) = (_v0, _a0, _j0, _psi0, _theta0, _dpsi0, _dtheta0)
        if self.avoid_method == 'sphere':
            rospy.logwarn('Sphere method is implemented yet')
            hypers = self.hypers_shrinked
        elif self.avoid_method == 'convex':
            rospy.logwarn('Convex method is implemented yet')
            hypers = self.hypers
        else:
            hypers = self.hypers
        for i in range(len(_x)):
            if i == len(_x) - 1:
                hyper = np.vstack(hypers[i])
                vT_, aT_, jT_, psiT_, dpsiT_, thetaT_, dthetaT_ = \
                    self.segment_generate(i + 1, hyper, x0=_x[i], _xT=_xef, v0=vT_, a0=aT_, j0=jT_, psi0=psiT_, theta0=thetaT_, dpsi0=dpsiT_, dtheta0=dthetaT_, RT=_Ref, endFlag=1)
            else:
                hyper = np.vstack(hypers[i])
                vT_, aT_, jT_, psiT_, dpsiT_, thetaT_, dthetaT_ = \
                    self.segment_generate(i + 1, hyper, x0=_x[i], _xT=_x[i + 1], v0=vT_, a0=aT_, j0=jT_, psi0=psiT_, theta0=thetaT_, dpsi0=dpsiT_, dtheta0=dthetaT_, RT=_Ref, endFlag=0)
        self.coeff = vertcat(*self.coeff)
        self.coeff_g = veccat(*self.coeff_g)
        self.g = vertcat(*self.g)
        self.lb = veccat(*self.lb)
        self.ub = veccat(*self.ub)

    def segment_generate(self, seg_i, hyper, x0, _xT, v0, a0, j0, psi0, theta0, dpsi0, dtheta0, RT, endFlag):
        dim_c = self.dim_c
        dim_a = self.dim_a
        dim_b = self.dim_b
        order_c = self.order_c
        order_a = self.order_a
        order_b = self.order_b
        
        gravity = self.dyncons.gravity
        da = self.dyncons.arm_length
        d0 = self.dyncons.d0
        d1 = self.dyncons.d1
        d2 = self.dyncons.d2

        g, lb, ub = self.g, self.lb, self.ub
        coeff, coeff_g = self.coeff, self.coeff_g
        cost = self.J
        d4x_cost, ddpsi_cost, ddtheta_cost = 0, 0, 0
        lambdaT = self.T_lambda
        lambda1, lambda2, lambda3 = self.d4x_lambda, self.ddpsi_lambda, self.ddtheta_lambda
        lambda4 = self.xef_lambda
        (v_max, a_max, dtheta_max, dpsi_max) = (
            self.dyncons.v_max, self.dyncons.a_max, self.dyncons.dtheta_max, self.dyncons.dpsi_max)
        (theta_min, theta_max) = (self.dyncons.theta_min, self.dyncons.theta_max)
        (theta1_min, theta1_max) = (self.dyncons.theta1_min, self.dyncons.theta1_max)
        (theta2_min, theta2_max) = (self.dyncons.theta2_min, self.dyncons.theta2_max)
        (hyperA, hyperb) = (hyper[:, 0:3], hyper[:, 3])
        C = Polynomial(order=order_c, dim=dim_c, char='c', seg_num=seg_i)
        A = Polynomial(order=order_a, dim=dim_a, char='a', seg_num=seg_i)
        B = Polynomial(order=order_b, dim=dim_b, char='b', seg_num=seg_i)

        C.set_c_coeff_g(x0=x0, xT=_xT, Tg=self.T_g)
        A.set_coeff_g()
        B.set_coeff_g()

        Tk = MX.sym('T|' + str(seg_i), 1)
        coeff += C.get_coeff()
        coeff += A.get_coeff()
        coeff += B.get_coeff()
        coeff += [Tk]

        coeff_g += C.get_coeff_g()
        coeff_g += A.get_coeff_g()
        coeff_g += B.get_coeff_g()
        coeff_g += [self.T_g]

        g += [Tk]
        lb += [1]
        ub += [1e9]

        N_seg = order_c * dim_c + order_a * dim_a + order_b * dim_b + 1
        N_psi = order_c * dim_c
        N_theta = order_c * dim_c + order_a * dim_a

        g += [coeff[0 * order_c + N_seg * (seg_i - 1)] - x0[0], 
              coeff[1 * order_c + N_seg * (seg_i - 1)] - x0[1],
              coeff[2 * order_c + N_seg * (seg_i - 1)] - x0[2]]
        g += [coeff[1 + 0 * order_c + N_seg * (seg_i - 1)] - v0[0], 
              coeff[1 + 1 * order_c + N_seg * (seg_i - 1)] - v0[1],
              coeff[1 + 2 * order_c + N_seg * (seg_i - 1)] - v0[2]]
        g += [2 * coeff[2 + 0 * order_c + N_seg * (seg_i - 1)] - a0[0],
              2 * coeff[2 + 1 * order_c + N_seg * (seg_i - 1)] - a0[1],
              2 * coeff[2 + 2 * order_c + N_seg * (seg_i - 1)] - a0[2]]
        g += [6 * coeff[3 + 0 * order_c + N_seg * (seg_i - 1)] - j0[0], 
              6 * coeff[3 + 1 * order_c + N_seg * (seg_i - 1)] - j0[1],
              6 * coeff[3 + 2 * order_c + N_seg * (seg_i - 1)] - j0[2]]

        lb += [0, 0, 0, 0, 0, 0, 0, 0, 0, -0.0, -0.0, -0.0]
        ub += [0, 0, 0, 0, 0, 0, 0, 0, 0,  0.0,  0.0,  0.0]
        g += [coeff[0 + N_psi + N_seg * (seg_i - 1)] - psi0, 
              coeff[1 + N_psi + N_seg * (seg_i - 1)] - dpsi0]
        lb += [0, 0]
        ub += [0, 0]
        g += [coeff[0 + 0 * order_b + N_theta + N_seg * (seg_i - 1)] - theta0[0],
              coeff[1 + 0 * order_b + N_theta + N_seg * (seg_i - 1)] - dtheta0[0],
			  coeff[0 + 1 * order_b + N_theta + N_seg * (seg_i - 1)] - theta0[1],
			  coeff[1 + 1 * order_b + N_theta + N_seg * (seg_i - 1)] - dtheta0[1]]
        lb += [0, 0, 0, 0]
        ub += [0, 0, 0, 0]

        dt = Tk / self.m_steps
        for i in range(self.m_steps):
            t = dt * i
            vk, ak, dthetak = [], [], []
            xk, thetak = [], []
            for j in range(dim_c):
                xjk = 0
                vjk = 0
                ajk = 0
                sjk = 0
                for k in range(order_c):
                    if k == 0:
                        xjk += coeff[k + order_c * j + N_seg * (seg_i - 1)]
                    else:
                        xjk += coeff[k + order_c * j + N_seg * (seg_i - 1)] * t ** k
                xk.append(xjk)
                for k in range(1, order_c):
                    if k == 1:
                        vjk += coeff[k + order_c * j + N_seg * (seg_i - 1)]
                    else:
                        vjk += k * coeff[k + order_c * j + N_seg * (seg_i - 1)] * t ** (k - 1)
                vk.append(vjk)
                for k in range(2, order_c):
                    if k == 2:
                        ajk += 2 * coeff[k + order_c * j + N_seg * (seg_i - 1)]
                    else:
                        ajk += k * (k - 1) * coeff[k + order_c * j + N_seg * (seg_i - 1)] * t ** (k - 2)
                ak.append(ajk)
                for k in range(order_c):
                    if k == 4:
                        sjk += k * (k - 1) * (k - 2) * (k - 3) * coeff[k + order_c * j + N_seg * (seg_i - 1)]
                    if k > 4:
                        sjk += k * (k - 1) * (k - 2) * (k - 3) * coeff[k + order_c * j + N_seg * (seg_i - 1)] * t ** (k - 4)
                d4x_cost += (sjk ** 2) * dt

            psijk = 0
            dpsijk = 0
            ddpsijk = 0
            for k in range(order_a):
                if k == 0:
                    psijk += coeff[k + N_psi + N_seg * (seg_i - 1)]
                else:
                    psijk += coeff[k + N_psi + N_seg * (seg_i - 1)] * t ** k
            psik = psijk
            for k in range(1, order_a):
                if k == 1:
                    dpsijk += coeff[k + N_psi + N_seg * (seg_i - 1)]
                else:
                    dpsijk += k * coeff[k + N_psi + N_seg * (seg_i - 1)] * t ** (k - 1)
            dpsik = dpsijk
            for k in range(order_a):
                if k == 2:
                    ddpsijk += k * (k - 1) * coeff[k + N_psi + N_seg * (seg_i - 1)]
                if k > 2:
                    ddpsijk += k * (k - 1) * coeff[k + N_psi + N_seg * (seg_i - 1)] * t ** (k - 2)
            ddpsi_cost += (ddpsijk ** 2) * dt

            for j in range(dim_b):
                thetajk = 0
                dthetajk = 0
                ddthetajk = 0
                for k in range(order_b):
                    if k == 0:
                        thetajk += coeff[k + N_theta + order_b * j + N_seg * (seg_i - 1)]
                    else:
                        thetajk += coeff[k + N_theta + order_b * j + N_seg * (seg_i - 1)] * t ** k
                thetak.append(thetajk)
                for k in range(1, order_b):
                    if k == 1:
                        dthetajk += coeff[k + N_theta + order_b * j + N_seg * (seg_i - 1)]
                    else:
                        dthetajk += k * coeff[k + N_theta + order_b * j + N_seg * (seg_i - 1)] * t ** (k - 1)
                dthetak.append(dthetajk)
                for k in range(order_b):
                    if k == 2:
                        ddthetajk += k * (k - 1) * coeff[k + N_theta + order_b * j + N_seg * (seg_i - 1)]
                    if k > 2:
                        ddthetajk += k * (k - 1) * coeff[k + N_theta + order_b * j + N_seg * (seg_i - 1)] * t ** (k - 2)
                ddtheta_cost += (ddthetajk ** 2) * dt

            cost += lambda1 * d4x_cost + lambda2 * ddpsi_cost + lambda3 * ddtheta_cost
            g += [vk[0], vk[1], vk[2], ak[0], ak[1], ak[2]]
            lb += [-v_max[0], -v_max[1], -v_max[2], -a_max[0], -a_max[1], -a_max[2]]
            ub += [ v_max[0],  v_max[1],  v_max[2],  a_max[0],  a_max[1],  a_max[2]]
            g += [dpsik, dthetak[0], dthetak[1]]
            lb += [-dpsi_max, -dtheta_max[0], -dtheta_max[1]]
            ub += [ dpsi_max,  dtheta_max[0],  dtheta_max[1]]
            g += [thetak[0], thetak[1]]
            lb += [theta1_min, theta2_min]
            ub += [theta1_max, theta2_max]
            if self.avoid_method == 'convex' and i % 2 == 0:
                xbm = np.array([d1*cos(thetak[0]), 0, - d0 - d1*sin(thetak[0])])
                xme = np.array([d2*cos(thetak[0]+thetak[1]), 0, -d2*sin(thetak[0]+thetak[1])])
                xmk = xk + xbm
                xek = xk + xbm + xme
                xa1k = xk + np.array([da, 0, 0])
                xa2k = xk + np.array([0, da, 0])
                xa3k = xk + np.array([-da, 0, 0])
                xa4k = xk + np.array([0, -da, 0])

                for j in range(hyperA.shape[0]):
                    g += [hyperA[j, 0] * xmk[0] + hyperA[j, 1] * xmk[1] + hyperA[j, 2] * xmk[2] - hyperb[j]]
                    g += [hyperA[j, 0] * xek[0] + hyperA[j, 1] * xek[1] + hyperA[j, 2] * xek[2] - hyperb[j]]
                    lb += [-1e9, -1e9]
                    ub += [0, 0]

                for j in range(hyperA.shape[0]):
                    g += [hyperA[j, 0] * xa1k[0] + hyperA[j, 1] * xa1k[1] + hyperA[j, 2] * xa1k[2] - hyperb[j]]
                    g += [hyperA[j, 0] * xa2k[0] + hyperA[j, 1] * xa2k[1] + hyperA[j, 2] * xa2k[2] - hyperb[j]]
                    g += [hyperA[j, 0] * xa3k[0] + hyperA[j, 1] * xa3k[1] + hyperA[j, 2] * xa3k[2] - hyperb[j]]
                    g += [hyperA[j, 0] * xa4k[0] + hyperA[j, 1] * xa4k[1] + hyperA[j, 2] * xa4k[2] - hyperb[j]]
                    lb += [-1e9, -1e9, -1e9, -1e9]
                    ub += [0, 0, 0, 0]

            elif self.avoid_method == 'sphere' and i % 2 == 0:
                for j in range(hyperA.shape[0]):
                    g += [hyperA[j, 0] * xk[0] + hyperA[j, 1] * xk[1] + hyperA[j, 2] * xk[2] - hyperb[j]]
                    lb += [-1e9]
                    ub += [0]

        cost += lambdaT * Tk

        xT_, vT_, aT_, jT_, psiT_, dpsiT_, thetaT_, dthetaT_ = [], [], [], [], [], [], [], []
        for j in range(dim_c):
            xTk = 0
            vTk = 0
            aTk = 0
            for k in range(order_c):
                if k == 0:
                    xTk += coeff[k + order_c * j + N_seg * (seg_i - 1)]
                else:
                    xTk += coeff[k + order_c * j + N_seg * (seg_i - 1)] * Tk ** k
            xT_.append(xTk)
            for k in range(1, order_c):
                if k == 1:
                    vTk += coeff[k + order_c * j + N_seg * (seg_i - 1)]
                else:
                    vTk += k * coeff[k + order_c * j + N_seg * (seg_i - 1)] * Tk ** (k - 1)
            vT_.append(vTk)
            for k in range(2, order_c):
                if k == 2:
                    aTk += 2 * coeff[k + order_c * j + N_seg * (seg_i - 1)]
                else:
                    aTk += k * (k - 1) * coeff[k + order_c * j + N_seg * (seg_i - 1)] * Tk ** (k - 2)
            aT_.append(aTk)
            for k in range(3, order_c):
                if k == 3:
                    jTk = 6 * coeff[k + order_c * j + N_seg * (seg_i - 1)]
                else:
                    jTk += k * (k - 1) * (k - 2) * coeff[k + order_c * j + N_seg * (seg_i - 1)] * Tk ** (k - 3)
            jT_.append(jTk)

        psiTk = 0
        dpsiTk = 0
        for k in range(order_a):
            if k == 0:
                psiTk += coeff[k + N_psi + N_seg * (seg_i - 1)]
            else:
                psiTk += coeff[k + N_psi + N_seg * (seg_i - 1)] * Tk ** k
        psiT_ = psiTk
        for k in range(1, order_a):
            if k == 1:
                dpsiTk = coeff[k + N_psi + N_seg * (seg_i - 1)]
            else:
                dpsiTk += k * coeff[k + N_psi + N_seg * (seg_i - 1)] * Tk ** (k - 1)
        dpsiT_ = dpsiTk
        
        for j in range(dim_b):
            thetaTk = 0
            dthetaTk = 0
            for k in range(order_b):
                if k == 0:
                    thetaTk += coeff[k + N_theta + order_b * j + N_seg * (seg_i - 1)]
                else:
                    thetaTk += coeff[k + N_theta + order_b * j + N_seg * (seg_i - 1)] * Tk ** k
            thetaT_.append(thetaTk)
            for k in range(1, order_b):
                if k == 1:
                    dthetaTk += coeff[k + N_theta + order_b * j + N_seg * (seg_i - 1)]
                else:
                    dthetaTk += k * coeff[k + N_theta + order_b * j + N_seg * (seg_i - 1)] * Tk ** (k - 1)
            dthetaT_.append(dthetaTk)

        if endFlag == 0:
            g += [xT_[0] - _xT[0], xT_[1] - _xT[1], xT_[2] - _xT[2]]
            lb += [0, 0, 0]
            ub += [0, 0, 0]

        elif endFlag == 1:
            s = [aT_[0], aT_[1], aT_[2] + gravity]
            norm_s = (aT_[0] ** 2 + aT_[1] ** 2 + (aT_[2] + gravity) ** 2) ** 0.5
            s[0] = s[0] / norm_s
            s[1] = s[1] / norm_s
            s[2] = s[2] / norm_s
            H2 = np.array([[1 - s[0] ** 2 / (1 + s[2]), - s[0] * s[1] / (1 + s[2]), s[0]],
                           [- s[0] * s[1] / (1 + s[2]), 1 - s[1] ** 2 / (1 + s[2]), s[1]],
                           [-s[0], - s[1], s[2]]])
            H2 = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])
            H1 = np.array([[cos(psiT_), -sin(psiT_), 0],
                           [sin(psiT_), cos(psiT_), 0],
                           [0, 0, 1]])
            Rb = np.matmul(H2, H1)
            xbe = np.array([d1*cos(thetaT_[0]) + d2*cos(thetaT_[0]+thetaT_[1]), 0, - d0 - d1*sin(thetaT_[0]) - d2*sin(thetaT_[0]+thetaT_[1])])

            Rbe = np.array([[cos(thetaT_[0]+thetaT_[1]), -sin(thetaT_[0]+thetaT_[1]), 0],
                            [0, 0, 1],
                            [-sin(thetaT_[0]+thetaT_[1]),-cos(thetaT_[0]+thetaT_[1]), 0]])
            Re = np.matmul(Rb, Rbe)
            Re_flat = Re.flatten()
            RT_flat = RT.flatten()
            xe = xT_ + np.matmul(Rb, xbe)

            g += [xe[0] - _xT[0], xe[1] - _xT[1], xe[2] - _xT[2]]
            lb += [0, 0, 0]
            ub += [0, 0, 0]
            g += [vT_[0], vT_[1], vT_[2]]
            lb += [0, 0, 0]
            ub += [0, 0, 0]
            g += [aT_[0], aT_[1], aT_[2]]
            lb += [0, 0, 0]
            ub += [0, 0, 0]
            g += [dthetaT_[0], dthetaT_[1], dpsiT_]
            lb += [0, 0, 0]
            ub += [0, 0, 0]

        self.g, self.lb, self.ub = g, lb, ub
        self.coeff, self.coeff_g = coeff, coeff_g
        self.J = cost
        Tk_ = Tk
        return vT_, aT_, jT_, psiT_, dpsiT_, thetaT_, dthetaT_

    def solve(self):
        self.nlp = {'f': self.J, 'x': self.coeff, 'g': self.g}
        solver = nlpsol('solver', 'ipopt', self.nlp, self.solver_options)
        solution = solver(x0=self.coeff_g, lbg=self.lb, ubg=self.ub)
        x_sol, g_sol = solution['x'].full().flatten(), solution['g'].full().flatten()
        self.x_sol = x_sol
        self.g_sol = g_sol
        return x_sol, g_sol

    def rearrange_result(self):
        x_sol = self.x_sol
        traj_num = self.traj_num
        coeff_ = {}
        dim_c = self.dim_c
        dim_a = self.dim_a
        dim_b = self.dim_b
        order_c = self.order_c
        order_a = self.order_a
        order_b = self.order_b
        N_seg = order_c * dim_c + order_a * dim_a + order_b * dim_b + 1
        N_psi = order_c * dim_c
        N_theta = order_c * dim_c + order_a * dim_a

        for i in range(traj_num):
            coeff_['seg|' + str(i + 1)] = {}
            coeff_['seg|' + str(i + 1)]['x'] = None
            coeff_['seg|' + str(i + 1)]['psi'] = None
            coeff_['seg|' + str(i + 1)]['theta'] = None
            coeff_['seg|' + str(i + 1)]['T'] = None

            C = Polynomial(order=order_c, dim=dim_c, char='c', seg_num=i + 1)
            A = Polynomial(order=order_a, dim=dim_a, char='a', seg_num=i + 1)
            B = Polynomial(order=order_b, dim=dim_b, char='b', seg_num=i + 1)

            coeff_c = C.rearrange_coeff(coeff=x_sol[      0 + i * N_seg :     N_psi + i * N_seg])
            coeff_a = A.rearrange_coeff(coeff=x_sol[  N_psi + i * N_seg :   N_theta + i * N_seg])
            coeff_b = B.rearrange_coeff(coeff=x_sol[N_theta + i * N_seg : N_seg - 1 + i * N_seg])
            coeff_T = x_sol[N_seg - 1 + i * N_seg]
            coeff_['seg|' + str(i + 1)]['x'] = coeff_c
            coeff_['seg|' + str(i + 1)]['psi'] = coeff_a
            coeff_['seg|' + str(i + 1)]['theta'] = coeff_b
            coeff_['seg|' + str(i + 1)]['T'] = coeff_T

        for i in range(traj_num):
            rospy.logwarn('seg_' + str(i) + ' time: ' + str(coeff_['seg|' + str(i + 1)]['T']))

        return coeff_
