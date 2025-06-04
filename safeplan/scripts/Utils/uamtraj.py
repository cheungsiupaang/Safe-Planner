import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import art3d
from numpy import cos, sin


class Uamtraj:
    def __init__(self, result, dynconstraint, dim, order):
        self.dim_c = dim['c']
        self.dim_a = dim['a']
        self.dim_b = dim['b']
        self.order_c = order['c']
        self.order_a = order['a']
        self.order_b = order['b']
        self.coeff = result
        self.delta_t = 0.02
        self.gravity = dynconstraint.gravity
        self.d0 = dynconstraint.d0
        self.d1 = dynconstraint.d1
        self.d2 = dynconstraint.d2
        self.de = dynconstraint.de
        self.arm_length = dynconstraint.arm_length
        self.T = np.empty(0)
        (self.x1, self.x2, self.x3, self.psi, self.theta1, self.theta2) = (
            np.empty(0), np.empty(0), np.empty(0), np.empty(0), np.empty(0), np.empty(0))
        (self.v1, self.v2, self.v3, self.dpsi, self.dtheta1, self.dtheta2) = (
            np.empty(0), np.empty(0), np.empty(0), np.empty(0), np.empty(0), np.empty(0))
        self.a1, self.a2, self.a3 = np.empty(0), np.empty(0), np.empty(0)
        self.rearrange_flat_output()
        self.Rb = []
        self.Re = []
        self.xb = []
        self.x0 = []
        self.xm = []
        self.xe = []
        self.rearrange_system_output()

    def rearrange_flat_output(self):
        data = self.coeff
        dt = self.delta_t
        order_c = self.order_c
        order_a = self.order_a
        order_b = self.order_b
        dim_c = self.dim_c
        dim_a = self.dim_a
        dim_b = self.dim_b
        c_tmp = np.zeros((dim_c, order_c))
        a_tmp = np.zeros((dim_a, order_a))
        b_tmp = np.zeros((dim_b, order_b))
        delta_T = 0.0
        for i in range(len(data)):
            key = 'seg|' + str(i + 1)
            step = int(data[key]['T'] / dt)
            for j in range(order_c):
                c_tmp[0, j] = data[key]['x'][j]
                c_tmp[1, j] = data[key]['x'][j + 1 * order_c]
                c_tmp[2, j] = data[key]['x'][j + 2 * order_c]
            for j in range(order_a):
                a_tmp[0, j] = data[key]['psi'][j]
            for j in range(order_b):
                b_tmp[0, j] = data[key]['theta'][j]
                b_tmp[1, j] = data[key]['theta'][j + order_b]
            c_tmp[0, :] = np.flip(c_tmp[0, :])
            c_tmp[1, :] = np.flip(c_tmp[1, :])
            c_tmp[2, :] = np.flip(c_tmp[2, :])
            a_tmp[0, :] = np.flip(a_tmp[0, :])
            b_tmp[0, :] = np.flip(b_tmp[0, :])
            b_tmp[1, :] = np.flip(b_tmp[1, :])
            for j in range(step):
                self.x1 = np.append(self.x1, np.polyval(c_tmp[0, :], j * dt))
                self.x2 = np.append(self.x2, np.polyval(c_tmp[1, :], j * dt))
                self.x3 = np.append(self.x3, np.polyval(c_tmp[2, :], j * dt))
                self.psi = np.append(self.psi, np.polyval(a_tmp[0, :], j * dt))
                self.theta1 = np.append(self.theta1, np.polyval(b_tmp[0, :], j * dt))
                self.theta2 = np.append(self.theta2, np.polyval(b_tmp[1, :], j * dt))
                self.T = np.append(self.T, delta_T + j * dt)
                self.v1 = np.append(self.v1, np.polyval(np.polyder(c_tmp[0, :]), j * dt))
                self.v2 = np.append(self.v2, np.polyval(np.polyder(c_tmp[1, :]), j * dt))
                self.v3 = np.append(self.v3, np.polyval(np.polyder(c_tmp[2, :]), j * dt))
                self.a1 = np.append(self.a1, np.polyval(np.polyder(c_tmp[0, :], 2), j * dt))
                self.a2 = np.append(self.a2, np.polyval(np.polyder(c_tmp[1, :], 2), j * dt))
                self.a3 = np.append(self.a3, np.polyval(np.polyder(c_tmp[2, :], 2), j * dt))
                
                self.dpsi = np.append(self.dpsi, np.polyval(np.polyder(a_tmp[0, :]), j * dt))
                self.dtheta1 = np.append(self.dtheta1, np.polyval(np.polyder(b_tmp[0, :]), j * dt))
                self.dtheta2 = np.append(self.dtheta2, np.polyval(np.polyder(b_tmp[1, :]), j * dt))

            delta_T += step * dt

    def rearrange_system_output(self):
        dt = self.delta_t
        T = self.T
        g = self.gravity
        d0, d1, d2 = self.d0, self.d1, self.d2
        x1, x2, x3 = self.x1, self.x2, self.x3
        psi = self.psi
        theta1, theta2 = self.theta1, self.theta2
        a1, a2, a3 = self.a1, self.a2, self.a3
        for i in range(len(T)):
            xb0 = np.array([0.0, 0.0, - d0])
            x0m = np.array([d1 * np.cos(theta1[i]), 0, -d1 * np.sin(theta1[i])])
            xme = np.array([d2 * np.cos(theta1[i] + theta2[i]), 0, -d2 * np.sin(theta1[i] + theta2[i])])
            xbm = xb0 + x0m
            xbe = xb0 + x0m + xme
            a_norm = np.sqrt(a1[i] ** 2 + a2[i] ** 2 + (a3[i] + g) ** 2)
            s1, s2, s3 = a1[i] / a_norm, a2[i] / a_norm, (a3[i] + g) / a_norm
            H2 = np.array([[1 - s1 ** 2 / (1 + s3), - s1 * s2 / (1 + s3), s1],
                           [- s1 * s2 / (1 + s3), 1 - s2 ** 2 / (1 + s3), s2],
                           [-s1, -s2, s3]])
            H1 = np.array([[np.cos(psi[i]), -np.sin(psi[i]), 0],
                           [np.sin(psi[i]), np.cos(psi[i]), 0],
                           [0, 0, 1]])
            Rb = np.dot(H1, H2)
            xb = np.array([x1[i], x2[i], x3[i]])
            x0 = xb + np.dot(Rb, xb0)
            xm = xb + np.dot(Rb, xbm)
            xe = xb + np.dot(Rb, xbe)
            Rbe = np.array([[cos(theta1[i] + theta2[i]), - sin(theta1[i] + theta2[i]), 0],
                            [0, 0, 1],
                            [-sin(theta1[i] + theta2[i]), - cos(theta1[i] + theta2[i]), 0]])
            Re = np.dot(Rb, Rbe)
            self.Rb.append(Rb)
            self.Re.append(Re)
            self.xb.append(xb)
            self.x0.append(x0)
            self.xm.append(xm)
            self.xe.append(xe)

    def plot2d(self):
        fig, axs = plt.subplots(2, 3, figsize=(8, 6))
        axs[0, 0].plot(self.T, self.dpsi)
        axs[0, 0].set_title(r'$\dot{\psi}$')
        axs[0, 0].get_xaxis().set_visible(False)

        axs[0, 1].plot(self.T, self.dtheta1)
        axs[0, 1].set_title(r'$\dot{\theta}_1$')
        axs[0, 1].get_xaxis().set_visible(False)

        axs[0, 2].plot(self.T, self.dtheta2)
        axs[0, 2].set_title(r'$\dot{\theta}_2$')
        axs[0, 2].get_xaxis().set_visible(False)

        axs[1, 0].plot(self.T, self.psi)
        axs[1, 0].set_title(r'$\psi$')

        axs[1, 1].plot(self.T, self.theta1)
        axs[1, 1].set_title(r'$\theta_1$')

        axs[1, 2].plot(self.T, self.theta2)
        axs[1, 2].set_title(r'$\theta_2$')

        fig, axs = plt.subplots(2, 3, figsize=(8, 6))
        axs[0, 0].plot(self.T, self.v1)
        axs[0, 0].set_title(r'$\dot{x}_1$')
        axs[0, 0].get_xaxis().set_visible(False)

        axs[0, 1].plot(self.T, self.v2)
        axs[0, 1].set_title(r'$\dot{x}_2$')
        axs[0, 1].get_xaxis().set_visible(False)

        axs[0, 2].plot(self.T, self.v3)
        axs[0, 2].set_title(r'$\dot{x}_3$')
        axs[0, 2].get_xaxis().set_visible(False)

        axs[1, 0].plot(self.T, self.x1)
        axs[1, 0].set_title(r'$x_1$')

        axs[1, 1].plot(self.T, self.x2)
        axs[1, 1].set_title(r'$x_2$')

        axs[1, 2].plot(self.T, self.x3)
        axs[1, 2].set_title(r'$x_3$')

        fig, axs = plt.subplots(2, 3, figsize=(8, 6))
        axs[0, 0].plot(self.T, self.a1)
        axs[0, 0].set_title(r'$\ddot{x}_1$')
        axs[0, 0].get_xaxis().set_visible(False)

        axs[0, 1].plot(self.T, self.a2)
        axs[0, 1].set_title(r'$\ddot{x}_2$')
        axs[0, 1].get_xaxis().set_visible(False)

        axs[0, 2].plot(self.T, self.a3)
        axs[0, 2].set_title(r'$\ddot{x}_3$')
        axs[0, 2].get_xaxis().set_visible(False)

        plt.show()

    def plot3d(self):
        xb = self.xb
        xe = self.xe
        Rb = self.Rb
        Re = self.Re
        arm_length = self.arm_length
        pb1 = np.array([arm_length * np.cos(np.pi / 6), -arm_length * np.sin(np.pi / 6), 0])
        pb2 = np.array([arm_length * np.cos(np.pi / 6), arm_length * np.sin(np.pi / 6), 0])
        pb3 = np.array([0, arm_length, 0])
        pb4 = np.array([-arm_length * np.cos(np.pi / 6), arm_length * np.sin(np.pi / 6), 0])
        pb5 = np.array([-arm_length * np.cos(np.pi / 6), -arm_length * np.sin(np.pi / 6), 0])
        pb6 = np.array([0, -arm_length, 0])

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_box_aspect((1, 1, 1))
        ax.quiver(0, 0, 0, 1, 0, 0, color='r', length=1)
        ax.quiver(0, 0, 0, 0, 1, 0, color='g', length=1)
        ax.quiver(0, 0, 0, 0, 0, 1, color='b', length=1)
        xb1, xb2, xb3 = np.empty(0), np.empty(0), np.empty(0)
        for i in range(len(xb)):
            xb1 = np.append(xb1, xb[i][0])
            xb2 = np.append(xb2, xb[i][1])
            xb3 = np.append(xb3, xb[i][2])
        ax.plot(xb1, xb2, xb3)

        count = 0
        for i in range(len(xb)):
            if count % 200 == 0:
                xbi = np.array([xb[i][0], xb[i][1], xb[i][2]])
                Rbi = Rb[i]
                propeller_radius = 0.12
                pw_head = np.array([np.dot(Rbi, pb) + xbi for pb in [pb1, pb2]])
                pw_other = np.array([np.dot(Rbi, pb) + xbi for pb in [pb3, pb4, pb5, pb6]])
                for pw in pw_head:
                    circle = plt.Circle((pw[0], pw[1]), propeller_radius, color='r', fill=False)
                    ax.plot([xbi[0], pw[0]], [xbi[1], pw[1]], [xbi[2], pw[2]], color='r')
                    ax.add_patch(circle)
                    art3d.pathpatch_2d_to_3d(circle, z=pw[2], zdir="z")
                for pw in pw_other:
                    circle = plt.Circle((pw[0], pw[1]), propeller_radius, color='k', fill=False)
                    ax.plot([xbi[0], pw[0]], [xbi[1], pw[1]], [xbi[2], pw[2]], color='k')
                    ax.add_patch(circle)
                    art3d.pathpatch_2d_to_3d(circle, z=pw[2], zdir="z")
                ax.plot([xb[i][0], xe[i][0]], [xb[i][1], xe[i][1]], [xb[i][2], xe[i][2]], color='k')
                ax.quiver(xe[i][0], xe[i][1], xe[i][2], Re[i][0][0], Re[i][1][0], Re[i][2][0], color='r',
                          length=0.3, normalize=False)
                ax.quiver(xe[i][0], xe[i][1], xe[i][2], Re[i][0][1], Re[i][1][1], Re[i][2][1], color='g',
                          length=0.3, normalize=False)
                ax.quiver(xe[i][0], xe[i][1], xe[i][2], Re[i][0][2], Re[i][1][2], Re[i][2][2], color='b',
                          length=0.3, normalize=False)
                ax.quiver(xb[i][0], xb[i][1], xb[i][2], Rb[i][0][2], Rb[i][1][2], Rb[i][2][2], color='k',
                          length=0.5, normalize=False)
            count += 1

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
        plt.show()
