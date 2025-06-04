import numpy as np
import yaml


class DynConstraint:
    def __init__(self, filename=""):
        self.x0, self.v0, self.a0, self.j0 = None, None, None, None
        self.psi0, self.dpsi0 = None, None
        self.theta0, self.dtheta0 = None, None
        self.xef, self.Ref = None, None
        self.v_max, self.a_max = None, None
        self.dtheta_max = None
        self.dpsi_max = None
        self.seg_x = None
        self.gravity = None
        self.arm_length = None
        self.de = None
        self.d0, self.d1, self.d2 = None, None, None
        self.lambda_de = None
        if filename:
            self.load_dynstraint(filename)

    def load_dynstraint(self, filename):
        print('Loading Traj from' + filename)
        with open(filename, 'r') as file:
            _dynstraint = yaml.load(file, Loader=yaml.FullLoader)
        self.v0 = _dynstraint['v0']
        self.a0 = _dynstraint['a0']
        self.j0 = _dynstraint['j0']
        self.psi0 = _dynstraint['psi0']
        self.theta0 = _dynstraint['theta0']
        self.dpsi0 = _dynstraint['dpsi0']
        self.dtheta0 = _dynstraint['dtheta0']
        _Ref = _dynstraint['Ref']
        self.Ref = np.array([[_Ref[0][0], _Ref[0][1], _Ref[0][2]],
                             [_Ref[1][0], _Ref[1][1], _Ref[1][2]],
                             [_Ref[2][0], _Ref[2][1], _Ref[2][2]]])
        self.v_max = _dynstraint['v_max']
        self.a_max = _dynstraint['a_max']
        self.dtheta_max = _dynstraint['dtheta_max']
        self.theta_min = _dynstraint['theta_min']
        self.theta_max = _dynstraint['theta_max']
        self.theta1_min = _dynstraint['theta1_min']
        self.theta1_max = _dynstraint['theta1_max']
        self.theta2_min = _dynstraint['theta2_min']
        self.theta2_max = _dynstraint['theta2_max']
        self.dpsi_max = _dynstraint['dpsi_max']
        self.arm_length = _dynstraint['arm_length']
        self.gravity = _dynstraint['gravity']
        self.de = _dynstraint['de']
        self.d0 = _dynstraint['d0']
        self.d1 = _dynstraint['d1']
        self.d2 = _dynstraint['d2']
        self.lambda_de = _dynstraint['lambda_de']
