from casadi import MX


class Polynomial:
    def __init__(self, order, dim, char, seg_num):
        self.coeff_g = None
        self.coeff = None
        self.order = order
        self.dim = dim
        self.set_coeff(char=char, seg_num=seg_num)

    def set_coeff(self, char=None, seg_num=0):
        _order = self.order
        _dim = self.dim
        coeff_ = []
        for i in range(_dim):
            for j in range(_order):
                coeff_.append(MX.sym(char+str(j)+str(i+1)+'|'+str(seg_num), 1))
        self.coeff = coeff_

    def set_coeff_g(self):
        _order = self.order
        _dim = self.dim
        coeff_g_ = []
        for i in range(_dim):
            for j in range(_order):
                coeff_g_ += [0]
        self.coeff_g = coeff_g_

    def set_c_coeff_g(self, x0, xT, Tg):
        _order = self.order
        _dim = self.dim
        coeff_g_ = []
        for i in range(_dim):
            for j in range(_order):
                if j == 0:
                    coeff_g_ += [x0[i]]
                elif j == 1:
                    coeff_g_ += [(xT[i] - x0[i])/Tg]
                else:
                    coeff_g_ += [0]
        self.coeff_g = coeff_g_

    def get_coeff(self):
        return self.coeff

    def get_coeff_g(self):
        return self.coeff_g

    def rearrange_coeff(self, coeff):
        _order = self.order
        _dim = self.dim
        coeff_ = []
        for i in range(_dim):
            for j in range(_order):
                coeff_.append(round(coeff[j+i*_order], 8))
        return coeff_
