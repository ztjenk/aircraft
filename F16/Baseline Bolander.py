import numpy as np
import matplotlib.pyplot as plt
import machupX as mx
import pandas as pd
from os.path import exists
import json
import windtunnelmodel as wind


def generate_data(params):
    alpha = params[0]
    beta = params[1]
    d_e = params[2]
    d_a = params[3]
    d_r = params[4]
    p = params[5]
    q = params[6]
    r = params[7]
    rates = [p, q, r]
    if nasa:
        case.set_state(alpha, beta, d_e, 0., 0., d_a, d_r,
                        0.35, p, q, r, 0.2, 1000., 0.)
        x = case.calc_forces(**forces_options)["F16"]
    else:
        my_scene.set_aircraft_state(state={"alpha": alpha,
                                            "beta": beta,
                                            "angular_rates": rates,
                                            "velocity": 222.5211})
        my_scene.set_aircraft_control_state(control_state={"elevator": d_e,
                                                            "aileron": d_a,
                                                            "rudder": d_r})
        x = my_scene.solve_forces(**forces_options)["F16"]["total"]
    fm = [x[CD], x['CS'], x['CL'], x['Cl'], x['Cm'], x['Cn']]
    return (*params, *fm)

def plot_model(x, y, color, ls, tag, marker, ax, label):
    first = True
    for i in range(len(y)):
        if tag[i] == "data":
            if first:
                ax.scatter(x, y[i], ec=color[i], fc="None",
                            marker=marker, label=label)
            else:
                ax.scatter(x, y[i], ec=color[i], fc="None", marker=marker)
            first = False
        else:
            ax.plot(x, y[i], color=color[i], linestyle=ls[i])
            
def remove_outliers(data, m=2.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else 0.
    return s < m

def _CL0_CLalpha(CLalpha_data, plot, skip_mask=False):
    alphas = CLalpha_data[:, 0]*np.pi/180.
    CL = CLalpha_data[:, 10]
    if skip_mask:
        out_mask = [True]*len(alphas)
    else:
        out_mask = remove_outliers(CL)
    [CL_alpha, CL0] = np.polyfit(alphas[out_mask], CL[out_mask], 1)
    if plot:
        x = alphas*180./np.pi
        y = [CL, CL0 + CL_alpha*alphas]
        color = ['0.0', '0.0']
        ls = ['-', '-']
        tag = ['data', 'not']
        marker = 'o'
        return [x, y, color, ls, tag, marker]
    return CL0, CL_alpha

def _CL_de(CLde_data, plot, skip_mask=False):
    CLde_p = np.array([x[10] for x in CLde_data if x[2] == 10.])
    CLde_m = np.array([x[10] for x in CLde_data if x[2] == -10.])
    de = np.deg2rad(10.)
    CL_de = (CLde_p - CLde_m)/(2.*de)
    if skip_mask:
        out_mask = [True]*len(CLde_p)
    else:
        out_mask = remove_outliers(CL_de)
    CL_de = np.average(CL_de[out_mask])

    CL0, CL_alpha = _CL0_CLalpha(np.array([x for x in CLde_data if x[2] == 0.]),
                                False)
    a0 = np.array([x[0] for x in CLde_data if x[2] == 0.])
    alpha = a0/180.*np.pi
    CL1 = CL0 + CL_alpha*alpha
    if plot:
        x = a0
        y = [CLde_p, CLde_m, CL1 + CL_de*de, CL1 + CL_de*-de]
        color = ['0.0']*4
        ls = ['--']*4
        tag = ['data']*2 + ['not']*2
        marker = 'v'
        return [x, y, color, ls, tag, marker]
    return CL_de

def _CL_qbar(CLqbar_data, plot, skip_mask=False):
    CLq_p = np.array([x[10] for x in CLqbar_data if x[6] == 30.*np.pi/180.])
    CLq_m = np.array([x[10] for x in CLqbar_data if x[6] == -30.*np.pi/180.])
    qbar = np.deg2rad(30.)*c_w/(2.*V)
    CL_qbar = (CLq_p - CLq_m)/(2.*qbar)
    if skip_mask:
        out_mask = [True]*len(CLq_p)
    else:
        out_mask = remove_outliers(CL_qbar)
    CL_qbar = np.average(CL_qbar[out_mask])

    CL0, CL_alpha = _CL0_CLalpha(np.array([x for x in CLqbar_data if x[6] == 0.]),
                                False)
    a0 = np.array([x[0] for x in CLqbar_data if x[6] == 0.])
    alpha = a0/180.*np.pi
    CL1 = CL0 + CL_alpha*alpha
    if plot:
        x = a0
        y = [CLq_p, CLq_m, CL1 + CL_qbar*qbar, CL1 + CL_qbar*-qbar]
        color = ['0.0']*4
        ls = [':']*4
        tag = ['data']*2 + ['not']*2
        marker = '^'
        return [x, y, color, ls, tag, marker]
    return CL_qbar

def _CS_beta(CSbeta_data, plot, skip_mask=False):
    betas = CSbeta_data[:, 1] * np.pi / 180.
    CS = CSbeta_data[:, 9]
    if skip_mask:
        out_mask = [True] * len(betas)
    else:
        out_mask = remove_outliers(CS)
    [CS_beta, CS0] = np.polyfit(betas[out_mask], CS[out_mask], 1)
    if plot:
        x = betas * 180. / np.pi
        y = [CS, CS0 + CS_beta * betas]
        color = ['0.0'] * 2
        ls = ['-'] * 2
        tag = ['data', 'not']
        marker = 'o'
        return [x, y, color, ls, tag, marker]
    return CS0, CS_beta

def _CS_da(CSda_data, plot, skip_mask=False):
    CS1 = np.array([x[9] for x in CSda_data if x[3] == 0.])
    CSda_p = np.array([x[9] for x in CSda_data if x[3] == 20.])
    da = np.deg2rad(20.)
    CS_da = (CSda_p - CS1) / da
    if skip_mask:
        out_mask = [True] * len(CS1)
    else:
        out_mask = remove_outliers(CS_da)
    CS_da = np.average(CS_da[out_mask])

    CS0, CS_beta = _CS_beta(np.array([x for x in CSda_data if x[3] == 0.]), False)
    b0 = np.array([x[1] for x in CSda_data if x[3] == 0.])
    beta = b0 * np.pi / 180.
    CS1 = CS0 + CS_beta * beta
    if plot:
        x = b0
        y = [CSda_p, CS1 + CS_da * da]
        color = ['0.0'] * 2
        ls = ['--'] * 2
        tag = ['data', 'not']
        marker = 'v'
        return [x, y, color, ls, tag, marker]
    return CS_da

def _CS_dr(CSdr_data, plot, skip_mask=False):
    CS1 = np.array([x[9] for x in CSdr_data if x[4] == 0.])
    CSdr_p = np.array([x[9] for x in CSdr_data if x[4] == 30.])
    dr = np.deg2rad(30.)
    CS_dr = (CSdr_p - CS1) / dr
    if skip_mask:
        out_mask = [True] * len(CS1)
    else:
        out_mask = remove_outliers(CS_dr)
    CS_dr = np.average(CS_dr[out_mask])

    CS0, CS_beta = _CS_beta(np.array([x for x in CSdr_data if x[4] == 0.]), False)
    b0 = np.array([x[1] for x in CSdr_data if x[4] == 0.])
    beta = b0 * np.pi / 180.
    CS1 = CS0 + CS_beta * beta
    if plot:
        x = b0
        y = [CSdr_p, CS1 + CS_dr * dr]
        color = ['0.0'] * 2
        ls = [':'] * 2
        tag = ['data', 'not']
        marker = '^'
        return [x, y, color, ls, tag, marker]
    return CS_dr

def _CS_rbar(CSr_data, plot, skip_mask=False):
    CSr_p = np.array([x[9] for x in CSr_data if x[7] == 30. * np.pi / 180.])
    CSr_m = np.array([x[9] for x in CSr_data if x[7] == -30. * np.pi / 180.])
    rbar = np.deg2rad(30.) * b_w / (2. * V)
    CS_rbar = (CSr_p - CSr_m) / (2. * rbar)
    if skip_mask:
        out_mask = [True] * len(CSr_p)
    else:
        out_mask = remove_outliers(CS_rbar)
    CS_rbar = np.average(CS_rbar[out_mask])

    CS1 = np.array([x[9] for x in CSr_data if x[7] == 0.])
    a0 = np.array([x[0] for x in CSr_data if x[7] == 0.])
    if plot:
        x = a0
        y = [CSr_p, CSr_m, CS1 + CS_rbar * rbar, CS1 + CS_rbar * -rbar]
        color = [0.0] * 4
        ls = ['-.'] * 4
        tag = ['data', 'data', 'not', 'not']
        marker = '<'
        return [x, y, color, ls, tag, marker]
    return CS_rbar

def _CS_pbar(CSp_data, plot, skip_mask=False):
    CS1 = np.array([x[9] for x in CSp_data if x[5] == 0.])
    CL1 = np.array([x[10] for x in CSp_data if x[5] == 0.])
    CSp_p = np.array([x[9] for x in CSp_data if x[5] == 90. * np.pi / 180.])
    CSp_m = np.array([x[9] for x in CSp_data if x[5] == -90. * np.pi / 180.])
    pbar = np.deg2rad(90.) * b_w / (2. * V)
    CS_pbar = (CSp_p - CSp_m) / (2. * pbar)
    if skip_mask:
        out_mask = [True] * len(CS1)
    else:
        out_mask = remove_outliers(CS_pbar)
    [CS_Lpbar, CS_pbar] = np.polyfit(CL1[out_mask], CS_pbar[out_mask], 1)

    CL0, CL_alpha = _CL0_CLalpha(np.array([x for x in CSp_data if x[5] == 0.]), False)
    a0 = np.array([x[0] for x in CSp_data if x[5] == 0.])
    alpha = a0 / 180. * np.pi
    CL1 = CL0 + CL_alpha * alpha
    if plot:
        x = a0
        y = [CSp_p,CSp_m,CS1 + (CS_Lpbar * CL1 + CS_pbar) * pbar,
            CS1 + (CS_Lpbar * CL1 + CS_pbar) * -pbar,]
        color = ['0.0'] * 4
        ls = [(0, (3, 5, 1, 5, 1, 5))] * 4
        tag = ['data', 'data', 'not', 'not']
        marker = '>'
        return [x, y, color, ls, tag, marker]
    return CS_pbar, CS_Lpbar

def _CD_de(CDde_data, plot, skip_mask=False):
    CD1 = np.array([x[8] for x in CDde_data if x[2] == 0.])
    CL1 = np.array([x[10] for x in CDde_data if x[2] == 0.])
    CDde_p = np.array([x[8] for x in CDde_data if x[2] == 10.])
    CDde_m = np.array([x[8] for x in CDde_data if x[2] == -10.])
    de = np.deg2rad(10.)
    CD_de = (CDde_p - CDde_m) / (2. * de)
    if skip_mask:
        out_mask = [True] * len(CD1)
    else:
        out_mask = remove_outliers(CD_de)
    [CD_Lde, CD_de] = np.polyfit(CL1[out_mask], CD_de[out_mask], 1)
    if skip_mask:
        out_mask = [True] * len(CD1)
    else:
        out_mask = remove_outliers(CD1)
    CD_de2 = np.average((CDde_p - CD1)[out_mask] / (de ** 2))

    CL0, CL_alpha = _CL0_CLalpha(np.array([x for x in CDde_data if x[2] == 0.]), 
                                 False)
    a0 = np.array([x[0] for x in CDde_data if x[2] == 0.])
    alpha = a0 / 180. * np.pi
    CL = CL0 + CL_alpha * alpha
    CD_0, CD_L, CD_L2 = _CD_polar(CDde_data, False)
    CD1 = CD_0 + CD_L * CL + CD_L2 * np.square(CL)
    if plot:
        x = CL1
        y = [CDde_p,CDde_m,CD1 + (CD_Lde * CL1 + CD_de) * de + CD_de2 * de ** 2,
            CD1 + (CD_Lde * CL1 + CD_de) * -de + CD_de2 * de ** 2,]
        color = ['0.0'] * 4
        ls = ['--'] * 4
        tag = ['data', 'data', 'not', 'not']
        marker = 'v' 
        return [x, y, color, ls, tag, marker]
    return CD_de, CD_Lde, CD_de

def _CD_polar(CDalpha_data, plot, skip_mask=False):
    CD = CDalpha_data[:, 8]
    CL = CDalpha_data[:, 10]
    if skip_mask:
        out_mask = [True] * len(CD)
    else:
        out_mask = remove_outliers(CD)
    out_mask *= remove_outliers(CL)
    out_mask *= (CD >= 0.)
    [CD_L2, CD_L, CD_0] = np.polyfit(CL[out_mask], CD[out_mask], 2)
    if plot:
        x = CL
        y = [CD, CD_0 + CD_L * CL + CD_L2 * np.square(CL)]
        color = ['0.0'] * 2
        ls = ['-'] * 2
        tag = ['data', 'not']
        marker = 'o'  
        coeffs = [CD_0, CD_L, CD_L2]
        return [x, y, color, ls, tag, marker, coeffs]
    return CD_0, CD_L, CD_L2

def _CD_Spolar(CDbeta_data, plot, skip_mask=False):
    CD = CDbeta_data[:, 8]
    CS = CDbeta_data[:, 9]
    if skip_mask:
        out_mask = [True] * len(CD)
    else:
        out_mask = remove_outliers(CD, m=1.5)
        out_mask *= remove_outliers(CS, m=1.5)
        out_mask *= (CD >= 0.)
    [CD_S2, CD_S, CD_0] = np.polyfit(CS[out_mask], CD[out_mask], 2)
    if plot:
        x = CS
        y = [CD, CD_0 + CD_S * CS + CD_S2 * np.square(CS)]
        color = ['0.3'] * 2
        ls = ['-'] * 2
        tag = ['data', 'not']
        marker = '<'
        coeffs = [CD_0, CD_S, CD_S2]
        return [x, y, color, ls, tag, marker, coeffs]
    return CD_0, CD_S, CD_S2

def _CD_pbar(CDp_data, plot, skip_mask=False):
    CD1 = np.array([x[8] for x in CDp_data if x[5] == 0.])
    CS1 = np.array([x[9] for x in CDp_data if x[5] == 0.])
    CDp_p = np.array([x[8] for x in CDp_data if x[5] == 90. * np.pi / 180.])
    CDp_m = np.array([x[8] for x in CDp_data if x[5] == -90. * np.pi / 180.])
    pbar = np.deg2rad(90.) * b_w / (2. * V)
    CD_pbar = (CDp_p - CDp_m) / (2. * pbar)
    if skip_mask:
        out_mask = [True] * len(CD1)
    else:
        out_mask = remove_outliers(CD_pbar)
        out_mask *= (CD1 >= 0.)
    [CD_Spbar, CD_pbar] = np.polyfit(CS1[out_mask], CD_pbar[out_mask], 1)

    # Shift to view accuracy of trends rather than discrepancy in CD_pbar
    CD1 = CDp_p[len(CD1) // 2]
    if plot:
        x = CS1
        y = [CDp_p,CDp_m,CD1 + (CD_Spbar * CS1 + CD_pbar) * pbar,
            CD1 + (CD_Spbar * CS1 + CD_pbar) * -pbar,]
        color = ['0.3'] * 4
        ls = ['--'] * 4
        tag = ['data', 'data', 'not', 'not']
        marker = '>' 
        return [x, y, color, ls, tag, marker]
    return CD_pbar, CD_Spbar

def _CD_rbar(CDr_data, plot, skip_mask=False):
    CD1 = np.array([x[8] for x in CDr_data if x[7] == 0.])
    CS1 = np.array([x[9] for x in CDr_data if x[7] == 0.])
    CDr_p = np.array([x[8] for x in CDr_data if x[7] == 90. * np.pi / 180.])
    CDr_m = np.array([x[8] for x in CDr_data if x[7] == -90. * np.pi / 180.])
    rbar = np.deg2rad(90.) * b_w / (2. * V)
    CD_rbar = (CDr_p - CDr_m) / (2. * rbar)
    if skip_mask:
        out_mask = [True] * len(CD1)
    else:
        out_mask = remove_outliers(CD_rbar)
    out_mask *= (CD1 >= 0.)
    [CD_Srbar, CD_rbar] = np.polyfit(CS1[out_mask], CD_rbar[out_mask], 1)

    # Shift to view accuracy of trends rather than discrepancy in CD_rbar
    CD1 = CDr_p[len(CD1) // 2]
    if plot:
        x = CS1
        y = [CDr_p,CDr_m,CD1 + (CD_Srbar * CS1 + CD_rbar) * rbar,
            CD1 + (CD_Srbar * CS1 + CD_rbar) * -rbar,]
        color = ['0.3'] * 4
        ls = ['--'] * 4
        tag = ['data', 'data', 'not', 'not']
        marker = '<'
        return [x, y, color, ls, tag, marker]
    return CD_rbar, CD_Srbar

def _CD_da(CDda_data, plot, skip_mask=False):
    CD1 = np.array([x[8] for x in CDda_data if x[3] == 0.])
    CS1 = np.array([x[9] for x in CDda_data if x[3] == 0.])
    CDda_p = np.array([x[8] for x in CDda_data if x[3] == 20.])
    CDda_m = np.array([x[8] for x in CDda_data if x[3] == -20.])
    da = np.deg2rad(20.)
    CD_da = (CDda_p - CDda_m) / (2. * da)
    if skip_mask:
        out_mask = [True] * len(CD1)
    else:
        out_mask = remove_outliers(CD_da)
        out_mask *= (CD1 >= 0.)
    [CD_Sda, CD_da] = np.polyfit(CS1[out_mask], CD_da[out_mask], 1)

    # Shift to view accuracy of trends rather than discrepancy in CD_da
    CD1 = CDda_p[len(CD1) // 2]
    if plot:
        x = CS1
        y = [CDda_p,CDda_m,CD1 + (CD_Sda * CS1 + CD_da) * da,
            CD1 + (CD_Sda * CS1 + CD_da) * -da,]
        color = ['0.3'] * 4
        ls = ['--'] * 4
        tag = ['data', 'data', 'not', 'not']
        marker = 'h'
        return [x, y, color, ls, tag, marker]
    return CD_da, CD_Sda

def _CD_dr(CDdr_data, plot, skip_mask=False):
    CD1 = np.array([x[8] for x in CDdr_data if x[4] == 0.])
    CS1 = np.array([x[9] for x in CDdr_data if x[4] == 0.])
    CDdr_p = np.array([x[8] for x in CDdr_data if x[4] == 30.])
    CDdr_m = np.array([x[8] for x in CDdr_data if x[4] == -30.])
    dr = np.deg2rad(30.)
    CD_dr = (CDdr_p - CDdr_m) / (2. * dr)
    if skip_mask:
        out_mask = [True] * len(CD1)
    else:
        out_mask = remove_outliers(CD_dr)
        out_mask *= (CD1 >= 0.)
    [CD_Sdr, CD_dr] = np.polyfit(CS1[out_mask], CD_dr[out_mask], 1)

    CD_0, CD_S, CD_S2 = _CD_Spolar(np.array([x for x in CDdr_data if x[4] == 0.]), 
                                   False)

    # Shift to view accuracy of trends rather than discrepancy in CD_dr
    CD1 = CDdr_p[len(CD1) // 2]
    if plot:
        x = CS1
        y = [CDdr_p,CDdr_m,CD1 + (CD_Sdr * CS1 + CD_dr) * dr,
            CD1 + (CD_Sdr * CS1 + CD_dr) * -dr,]
        color = ['0.3'] * 4
        ls = [(0, (3, 5, 1, 5, 1, 5))] * 4
        tag = ['data', 'data', 'not', 'not']
        marker = 'd' 
        return [x, y, color, ls, tag, marker]
    return CD_dr, CD_Sdr

def _CD_qbar(CDq_data, plot, skip_mask=False):
    CD1 = np.array([x[8] for x in CDq_data if x[6] == 0.])
    CL1 = np.array([x[10] for x in CDq_data if x[6] == 0.])
    CDq_p = np.array([x[8] for x in CDq_data if x[6] == 30. * np.pi / 180.])
    CDq_m = np.array([x[8] for x in CDq_data if x[6] == -30. * np.pi / 180.])
    qbar = np.deg2rad(30.) * c_w / (2. * V)
    CD_qbar = (CDq_p - CDq_m) / (2. * qbar)
    if skip_mask:
        out_mask = [True] * len(CD1)
    else:
        out_mask = remove_outliers(CD_qbar)
        out_mask *= (CD1 >= 0.)
    [CD_L2qbar, CD_Lqbar, CD_qbar] = np.polyfit(CL1[out_mask], CD_qbar[out_mask], 2)

    CL0, CL_alpha = _CL0_CLalpha(np.array([x for x in CDq_data if x[6] == 0.]), False)
    a0 = np.array([x[0] for x in CDq_data if x[6] == 0.])
    alpha = a0 / 180. * np.pi
    CL = CL0 + CL_alpha * alpha
    CD_0, CD_L, CD_L2 = _CD_polar(np.array([x for x in CDq_data if x[6] == 0.]),
                                   False)
    CD1 = CD_0 + CD_L * CL + CD_L2 * np.square(CL)
    if plot:
        x = CL1
        y = [CDq_p,CDq_m,
             CD1 + (CD_L2qbar * np.square(CL1) + CD_Lqbar * CL1 + CD_qbar) * qbar,
            CD1 + (CD_L2qbar * np.square(CL1) + CD_Lqbar * CL1 + CD_qbar) * -qbar,]
        color = ['0.0'] * 4
        ls = [':'] * 4
        tag = ['data', 'data', 'not', 'not']
        marker = '^' 
        return [x, y, color, ls, tag, marker]
    return CD_qbar, CD_Lqbar, CD_L2qbar

def _Cl_beta(Clbeta_data, plot, skip_mask=False):
    betas = Clbeta_data[:, 1] * np.pi / 180.
    Cl = Clbeta_data[:, 11]
    if skip_mask:
        out_mask = [True] * len(betas)
    else:
        out_mask = remove_outliers(Cl)
    [Cl_beta, Cl0] = np.polyfit(betas[out_mask], Cl[out_mask], 1)

    b0 = betas * 180. / np.pi
    if plot:
        x = b0
        y = [Cl, Cl0 + Cl_beta * betas]
        color = ['0.0'] * 2
        ls = ['-'] * 2
        tag = ['data', 'not']
        marker = 'o'
        return [x, y, color, ls, tag, marker]
    return Cl0, Cl_beta

def _Cl_pbar(Clp_data, plot, skip_mask=False):
    Cl1 = np.array([x[11] for x in Clp_data if x[5] == 0.])
    Clp_p = np.array([x[11] for x in Clp_data if x[5] == 90. * np.pi / 180.])
    Clp_m = np.array([x[11] for x in Clp_data if x[5] == -90. * np.pi / 180.])
    pbar = np.deg2rad(90.) * b_w / (2. * V)
    if skip_mask:
        out_mask = [True] * len(Cl1)
    else:
        out_mask = remove_outliers(Clp_p)
    Cl_pbar = np.average((Clp_p - Clp_m)[out_mask] / (2. * pbar))

    a0 = np.array([x[0] for x in Clp_data if x[5] == 0.])
    Cl1 = np.zeros(len(Cl1))
    if plot:
        x = a0
        y = [Clp_p, Clp_m, Cl1 + Cl_pbar * pbar, Cl1 + Cl_pbar * -pbar]
        color = ['0.0'] * 4
        ls = ['--'] * 4
        tag = ['data', 'data', 'not', 'not']
        marker = '<' 
        return [x, y, color, ls, tag, marker]
    return Cl_pbar

def _Cl_rbar(Clr_data, plot, skip_mask=False):
    Cl1 = np.array([x[11] for x in Clr_data if x[7] == 0.])
    CL1 = np.array([x[10] for x in Clr_data if x[7] == 0.])
    Clr_p = np.array([x[11] for x in Clr_data if x[7] == 90. * np.pi / 180.])
    Clr_m = np.array([x[11] for x in Clr_data if x[7] == -90. * np.pi / 180.])
    rbar = np.deg2rad(90.) * b_w / (2. * V)
    Cl_rbar=(Clr_p-Clr_m)/(2.*rbar)
    if skip_mask:
        out_mask = [True] * len(Cl1)
    else:
        out_mask = remove_outliers(Clr_p)
    [Cl_Lrbar,Cl_rbar]= np.polyfit(CL1[out_mask],Cl_rbar[out_mask],1)

    a0 = np.array([x[0] for x in Clr_data if x[7] == 0.])
    Cl1 = np.zeros(len(a0))
    [CL0,CL_alpha]=_CL0_CLalpha(np.array([xforxinClr_dataifx[7]==0.]),
                                False)
    CL1=CL0+CL_alpha*a0*np.pi/180.
    if plot:
        x = a0
        y = [Clr_p,Clr_m,Cl1+(Cl_Lrbar*CL1+Cl_rbar)*rbar,
             Cl1+(Cl_Lrbar*CL1+Cl_rbar)*-rbar]
        color = ['0.0'] * 4
        ls = ['--'] * 4
        tag = ['data', 'data', 'not', 'not']
        marker = '>' 
        return [x, y, color, ls, tag, marker]
    return Cl_rbar, Cl_Lrbar

def _Cl_da(Clda_data, plot, skip_mask=False):
    Cl1 = np.array([x[11] for x in Clda_data if x[3] == 0.])
    Clda_p = np.array([x[11] for x in Clda_data if x[3] == 20.])
    da = np.deg2rad(20.)
    if skip_mask:
        out_mask = [True] * len(Cl1)
    else:
        out_mask = remove_outliers(Cl1)
    Cl_da = np.average((Clda_p - Cl1)[out_mask] / da)

    b0 = np.array([x[1] for x in Clda_data if x[3] == 0.])
    [Cl0, Cl_beta] = _Cl_beta(np.array([x for x in Clda_data if x[3] == 0.]), False)
    Cl1 = Cl0 + Cl_beta * b0 * np.pi / 180.
    if plot:
        x = b0
        y = [Clda_p, Cl1 + Cl_da * np.full(len(Cl1), da)]
        color = ['0.0'] * 2
        ls = ['--'] * 2
        tag = ['data', 'not']
        marker = 'v' 
        return [x, y, color, ls, tag, marker]
    return Cl_da

def _Cl_dr(Cldr_data, plot, skip_mask=False):
    Cl1 = np.array([x[11] for x in Cldr_data if x[4] == 0.])
    Cldr_p = np.array([x[11] for x in Cldr_data if x[4] == 30.])
    dr = np.deg2rad(30.)
    if skip_mask:
        out_mask = [True] * len(Cl1)
    else:
        out_mask = remove_outliers(Cl1)
    Cl_dr = np.average((Cldr_p - Cl1)[out_mask] / dr)

    b0 = np.array([x[1] for x in Cldr_data if x[4] == 0.])
    [Cl0, Cl_beta] = _Cl_beta(np.array([x for x in Cldr_data if x[4] == 0.]), False)
    Cl1 = Cl0 + Cl_beta * b0 * np.pi / 180.
    if plot:
        x = b0
        y = [Cldr_p, Cl1 + Cl_dr * np.full(len(Cl1), dr)]
        color = [0.0] * 2
        ls = [':'] * 2
        tag = [data, not]
        marker = '^'  # Placeholder for marker
        return [x, y, color, ls, tag, marker]
    return Cl_dr

def _Cm0_Cmalpha(Cmalpha_data, plot, skip_mask=False):
    alphas = Cmalpha_data[:, 0] * np.pi / 180.
    Cm = Cmalpha_data[:, 12]
    if skip_mask:
        out_mask = [True] * len(alphas)
    else:
        out_mask = alphas <= 0.  # Effect of LEV
        out_mask[0] = False  # using data points centered around zero-lift alpha
    [Cm_alpha, Cm0] = np.polyfit(alphas[out_mask], Cm[out_mask], 1)

    a0 = Cmalpha_data[:, 0]
    if plot:
        x = a0
        y = [Cm, Cm0 + Cm_alpha * alphas]
        color = ['0.0'] * 2
        ls = ['-'] * 2
        tag = ['data', 'not']
        marker = 'o' 
        return [x, y, color, ls, tag, marker]
    return Cm0, Cm_alpha

def _Cm_qbar(Cmq_data, plot, skip_mask=False):
    Cm1 = np.array([x[12] for x in Cmq_data if x[6] == 0.])
    Cmq_p = np.array([x[12] for x in Cmq_data if x[6] == 30. * np.pi / 180.])
    Cmq_m = np.array([x[12] for x in Cmq_data if x[6] == -30. * np.pi / 180.])
    qbar = np.deg2rad(30.) * c_w / (2. * V)
    if skip_mask:
        out_mask = [True] * len(Cm1)
    else:
        out_mask = remove_outliers(Cm1)
    Cm_qbar = np.average((Cmq_p - Cmq_m)[out_mask] / (2. * qbar))

    a0 = np.array([x[0] for x in Cmq_data if x[6] == 0.])
    [Cm0, Cm_alpha] = _Cm0_Cmalpha(np.array([x for x in Cmq_data if x[6] == 0.]), 
                                   False)
    Cm1 = Cm0 + Cm_alpha * a0 * np.pi / 180.
    if plot:
        x = a0
        y = [Cmq_p, Cmq_m, Cm1 + Cm_qbar * qbar, Cm1 + Cm_qbar * -qbar]
        color = ['0.0'] * 4
        ls = ['--'] * 4
        tag = ['data', 'data', 'not', 'not']
        marker = 'v' 
        return [x, y, color, ls, tag, marker]
    return Cm_qbar

def _Cm_de(Cmde_data, plot, skip_mask=False):
    Cm1 = np.array([x[12] for x in Cmde_data if x[2] == 0.])
    Cmde_p = np.array([x[12] for x in Cmde_data if x[2] == 10.])
    Cmde_m = np.array([x[12] for x in Cmde_data if x[2] == -10.])
    de = np.deg2rad(10.)
    if skip_mask:
        out_mask = [True] * len(Cm1)
    else:
        out_mask = remove_outliers(Cm1)
    Cm_de = np.average((Cmde_p - Cmde_m)[out_mask] / (2. * de))

    a0 = np.array([x[0] for x in Cmde_data if x[2] == 0.])
    [Cm0, Cm_alpha] = _Cm0_Cmalpha(np.array([x for x in Cmde_data if x[2] == 0.]), 
                                   False)
    Cm1 = Cm0 + Cm_alpha * a0 * np.pi / 180.
    if plot:
        x = a0
        y = [Cmde_p, Cmde_m, Cm1 + Cm_de * de, Cm1 + Cm_de * -de]
        color = ['0.0'] * 4
        ls = [':'] * 4
        tag = ['data', 'data', 'not', 'not']
        marker = '^' 
        return [x, y, color, ls, tag, marker]
    return Cm_de

def _Cn_beta(Cnbeta_data, plot, skip_mask=False):
    betas = Cnbeta_data[:, 1] * np.pi / 180.
    Cn = Cnbeta_data[:, 13]
    if skip_mask:
        out_mask = [True] * len(Cn)
    else:
        out_mask = remove_outliers(Cn)
    [Cn_beta, Cn0] = np.polyfit(betas[out_mask], Cn[out_mask], 1)

    b0 = betas * 180. / np.pi
    if plot:
        x = b0
        y = [Cn, Cn0 + Cn_beta * betas]
        color = ['0.0'] * 2
        ls = ['-'] * 2
        tag = ['data', 'not']
        marker = 'o' 
        return [x, y, color, ls, tag, marker]
    return Cn0, Cn_beta

def _Cn_pbar(Cnp_data, plot, skip_mask=False):
    Cn1 = np.array([x[13] for x in Cnp_data if x[5] == 0.])
    Cnp_p = np.array([x[13] for x in Cnp_data if x[5] == 90. * np.pi / 180.])
    Cnp_m = np.array([x[13] for x in Cnp_data if x[5] == -90. * np.pi / 180.])
    CL1 = np.array([x[10] for x in Cnp_data if x[5] == 0.])
    pbar = np.deg2rad(90.) * b_w / (2. * V)
    Cn_pbar = (Cnp_p - Cnp_m) / (2. * pbar)
    if skip_mask:
        out_mask = [True] * len(Cn1)
    else:
        out_mask = remove_outliers(Cn_pbar)
    [Cn_Lpbar, Cn_pbar] = np.polyfit(CL1[out_mask], Cn_pbar[out_mask], 1)

    a0 = np.array([x[0] for x in Cnp_data if x[5] == 0.])
    Cn1 = np.zeros(len(Cn1))
    if plot:
        x = a0
        y = [Cnp_p, Cnp_m, Cn1 + (Cn_Lpbar * CL1 + Cn_pbar) * pbar,
             Cn1 + (Cn_Lpbar * CL1 + Cn_pbar) * -pbar]
        color = ['0.0'] * 4
        ls = [':'] * 4
        tag = ['data', 'data', 'not', 'not']
        marker = '^' 
        return [x, y, color, ls, tag, marker]
    return Cn_pbar, Cn_Lpbar

def _Cn_rbar(Cnr_data, plot, skip_mask=False):
    Cn1 = np.array([x[13] for x in Cnr_data if x[7] == 0.])
    Cnr_p = np.array([x[13] for x in Cnr_data if x[7] == 30. * np.pi / 180.])
    Cnr_m = np.array([x[13] for x in Cnr_data if x[7] == -30. * np.pi / 180.])
    rbar = np.deg2rad(30.) * b_w / (2. * V)
    if skip_mask:
        out_mask = [True] * len(Cn1)
    else:
        out_mask = remove_outliers(Cnr_p)
    Cn_rbar = np.average((Cnr_p - Cnr_m)[out_mask] / (2. * rbar))

    a0 = np.array([x[0] for x in Cnr_data if x[7] == 0.])
    Cn1 = np.zeros(len(Cn1))
    if plot:
        x = a0
        y = [Cnr_p, Cnr_m, Cn1 + Cn_rbar * rbar, Cn1 + Cn_rbar * -rbar]
        color = ['0.0'] * 4
        ls = ['-.'] * 4
        tag = ['data', 'data', 'not', 'not']
        marker = '<' 
        return [x, y, color, ls, tag, marker]
    return Cn_rbar




####
def _Cn_da(Cnda_data, plot, skip_mask=False):
Cn1 = np.array([x[13] for x in Cnda_data if x[3] == 0.])
Cnda_p = np.array([x[13] for x in Cnda_data if x[3] == 20.])322
CL1 = np.array([x[10] for x in Cnda_data if x[3] == 0.])
da = np.deg2rad(20.)
Cn_da = (Cnda_p - Cn1)/da
if skip_mask:
out_mask = [True]*len(Cn1)
else:
out_mask = remove_outliers(Cn_da)
[Cn_Lda, Cn_da] = np.polyfit(CL1[out_mask], Cn_da[out_mask], 1)
a0 = np.array([x[0] for x in Cnda_data if x[3] == 0.])
Cn1 = np.zeros(len(Cn1))
if plot:
x = a0
y = [Cnda_p, Cn1 + (Cn_Lda*CL1 + Cn_da)*da]
color = [
0.0
]*2
ls = [(0, (3, 5, 1, 5, 1, 5))]*2
tag = [
data
, 
not
]
marker = 
>

return [x, y, color, ls, tag, marker]
return Cn_da, Cn_Lda
def _Cn_dr(Cndr_data, plot, skip_mask=False):
Cn1 = np.array([x[13] for x in Cndr_data if x[4] == 0.])
Cndr_p = np.array([x[13] for x in Cndr_data if x[4] == 30.])
dr = np.deg2rad(30.)
if skip_mask:
out_mask = [True]*len(Cn1)
else:
out_mask = remove_outliers(Cndr_p)
Cn_dr = np.average((Cndr_p - Cn1)[out_mask]/dr)
b0 = np.array([x[1] for x in Cndr_data if x[4] == 0.])
[Cn0, Cn_beta] = _Cn_beta(np.array([x for x in Cndr_data if x[4] == 0.]), False)
Cn1 = Cn0 + Cn_beta*b0*np.pi/180.
if plot:
x = b0
y = [Cndr_p, Cn1 + Cn_dr*dr]
color = [
0.0
]*2
ls = [
--
]*2
tag = [
data
, 
not
]
marker = 
v

return [x, y, color, ls, tag, marker]
return Cn_dr
def create_database():
data = np.zeros((N_alpha*N_other_a + N_beta*N_other_b, 14))
params = np.zeros(8)
zz = 0
#len(alpha_range) 1a
for a in alpha_range:
params[0] = a
data[zz, :] = generate_data(params)
zz += 1
params[0] = 0.
#len(beta_range) 1b323
for b in beta_range:
params[1] = b
data[zz, :] = generate_data(params)
zz += 1
params[1] = 0.
#len(de_range)*len(a_range) len(de_range)*1a
for e in de_range:
params[2] = e
for a in alpha_range:
params[0] = a
data[zz, :] = generate_data(params)
zz += 1
params[2] = 0.
params[0] = 0.
for da in da_range:
params[3] = da
#len(beta_range)
for b in beta_range:
params[1] = b
data[zz, :] = generate_data(params)
zz += 1
params[1] = 0.
#len(alpha_range)
for a in alpha_range:
params[0] = a
data[zz, :] = generate_data(params)
zz += 1
params[0] = 0.
params[3] = 0.
#len(beta_range)
for dr in dr_range:
params[4] = dr
for b in beta_range:
params[1] = b
data[zz, :] = generate_data(params)
zz += 1
params[1] = 0.
params[4] = 0.
#len(p_range)*(len(alpha_range) + len(beta_range))
for p in p_range:
params[5] = p
for a in alpha_range:
params[0] = a
data[zz, :] = generate_data(params)
zz += 1
params[0] = 0.
for b in beta_range:
params[1] = b
data[zz, :] = generate_data(params)
zz += 1
params[1] = 0.
params[5] = 0.
for q in q_range:
params[6] = q
for a in alpha_range:
params[0] = a324
data[zz, :] = generate_data(params)
zz += 1
params[6] = 0.
#len(r_range)*(len(alpha_range) + len(beta_range))
for r in r_range:
params[7] = r
for a in alpha_range:
params[0] = a
data[zz, :] = generate_data(params)
zz += 1
params[0] = 0.
for b in beta_range:
params[1] = b
data[zz, :] = generate_data(params)
zz += 1
params[1] = 0.
params[7] = 0.
return data
def find_model(database):
plot = False
df = pd.DataFrame(database,
columns = [
Alpha
,
Beta
,
d_e
, 
d_a
, 
d_r
, 
p
, 
q
, 
r
,
CD
, 
CS
, 
CL
, 
Cl
, 
Cm
, 
Cn
])
CLalpha_data = df.loc[df[
Beta
] + df[
d_e
] + df[
d_a
] + df[
d_r
] +
df[
p
] + df[
q
] + df[
r
] == 0].to_numpy()
CL_0, CL_alpha = _CL0_CLalpha(CLalpha_data, plot)
CLde_data = df.loc[df[
Beta
] + df[
d_a
] + df[
d_r
] +
df[
p
] + df[
q
] + df[
r
] == 0].to_numpy()
CL_de = _CL_de(CLde_data, plot)
CLqbar_data = df.loc[df[
Beta
] + df[
d_e
] + df[
d_a
] + df[
d_r
] +
df[
p
] + df[
r
] == 0].to_numpy()
CL_qbar = _CL_qbar(CLqbar_data, plot)
CSbeta_data = df.loc[((df[
Alpha
] + df[
d_e
] + df[
d_a
] + df[
d_r
] +
df[
p
] + df[
q
] + df[
r
] == 0) &
(df[
Alpha
] == 0.))].sort_values(by=[
Beta
]).to_numpy()
CS_0, CS_beta = _CS_beta(CSbeta_data, plot)
CSda_data = df.loc[((df[
Alpha
] + df[
d_e
] + df[
d_r
] +
df[
p
] + df[
q
] + df[
r
] == 0) &
(df[
Alpha
] == 0.))].sort_values(by=[
Beta
]).to_numpy()
CS_da = _CS_da(CSda_data, plot)
CSdr_data = df.loc[((df[
Alpha
] + df[
d_e
] + df[
d_a
] +
df[
p
] + df[
q
] + df[
r
] == 0) &
(df[
Alpha
] == 0.))].sort_values(by=[
Beta
]).to_numpy()
CS_dr = _CS_dr(CSdr_data, plot)
CSr_data = df.loc[((df[
Beta
] + df[
d_e
] + df[
d_a
] +
df[
p
] + df[
q
] + df[
d_r
] == 0))].to_numpy()
CS_rbar = _CS_rbar(CSr_data, plot)
CSp_data = df.loc[((df[
Beta
] + df[
d_e
] + df[
d_a
] + df[
d_r
] +
df[
q
] + df[
r
] == 0))].to_numpy()
CS_pbar, CS_Lpbar = _CS_pbar(CSp_data, plot, skip_mask=True)
CDde_data = df.loc[((df[
Beta
] + df[
p
] + df[
d_a
] + df[
d_r
] +
df[
q
] + df[
r
] == 0))].to_numpy()
CD_de, CD_Lde, CD_de2 = _CD_de(CDde_data, plot)
CD_0, CD_L, CD_L2 = _CD_polar(CLalpha_data, plot)
CD_S2 = _CD_Spolar(CSbeta_data, plot)[2]325
CD_qbar, CD_Lqbar, CD_L2qbar = _CD_qbar(CLqbar_data, plot)
CDp_data = df.loc[((df[
Alpha
] + df[
d_e
] + df[
d_a
] + df[
d_r
] +
df[
q
] + df[
r
] == 0) &
(df[
Alpha
] == 0.))].to_numpy()
CDr_data = df.loc[((df[
Alpha
] + df[
d_e
] + df[
d_a
] + df[
d_r
] +
df[
q
] + df[
p
] == 0) &
(df[
Alpha
] == 0.))].to_numpy()
CD_pbar, CD_Spbar = _CD_pbar(CDp_data, plot, skip_mask=True)
CD_rbar, CD_Srbar = _CD_rbar(CDr_data, plot)
CD_da, CD_Sda = _CD_da(CSda_data, plot)
CD_dr, CD_Sdr = _CD_dr(CSdr_data, plot)
Cl_0, Cl_beta = _Cl_beta(CSbeta_data, plot)
Cl_pbar = _Cl_pbar(CSp_data, plot)
Cl_rbar, Cl_Lrbar = _Cl_rbar(CSr_data, plot)
Cl_da = _Cl_da(CSda_data, plot)
Cl_dr = _Cl_dr(CSdr_data, plot)
Cm_0, Cm_alpha = _Cm0_Cmalpha(CLalpha_data, plot)
Cm_qbar = _Cm_qbar(CLqbar_data, plot)
Cm_de = _Cm_de(CLde_data, plot)
Cn_0, Cn_beta = _Cn_beta(CSbeta_data, plot)
Cn_pbar, Cn_Lpbar = _Cn_pbar(CSp_data, plot, skip_mask=True)
Cn_rbar = _Cn_rbar(CSr_data, plot)
Cnda_data = df.loc[((df[
Beta
] + df[
p
] + df[
d_e
] + df[
d_r
] +
df[
q
] + df[
r
] == 0))].to_numpy()
Cn_da, Cn_Lda = _Cn_da(Cnda_data, plot)
Cn_dr = _Cn_dr(CSdr_data, plot)
coeff_database = {"CL": {"CL_0": CL_0,
"CL_alpha": CL_alpha,
"CL_qbar": CL_qbar,
"CL_de": CL_de},
"CS": {"CS_beta": CS_beta,
"CS_pbar": CS_pbar,
"CS_Lpbar": CS_Lpbar,
"CS_rbar": CS_rbar,
"CS_da": CS_da,
"CS_dr": CS_dr},
"CD": {"CD_0": CD_0,
"CD_L": CD_L,
"CD_L2": CD_L2,
"CD_S2": CD_S2,
"CD_Spbar": CD_Spbar,
"CD_qbar": CD_qbar,
"CD_Lqbar": CD_Lqbar,
"CD_L2qbar": CD_L2qbar,
"CD_Srbar": CD_Srbar,
"CD_de": CD_de,
"CD_Lde": CD_Lde,
"CD_de2": CD_de2,
"CD_Sda": CD_Sda,
"CD_Sdr": CD_Sdr},
"Cell": {"Cl_beta": Cl_beta,
"Cl_pbar": Cl_pbar,
"Cl_rbar": Cl_rbar,
"Cl_Lrbar": Cl_Lrbar,
"Cl_da": Cl_da,
"Cl_dr": Cl_dr},326
"Cm": {"Cm_0": Cm_0,
"Cm_alpha": Cm_alpha,
"Cm_qbar": Cm_qbar,
"Cm_de": Cm_de},
"Cn": {"Cn_beta": Cn_beta,
"Cn_pbar": Cn_pbar,
"Cn_Lpbar": Cn_Lpbar,
"Cn_rbar": Cn_rbar,
"Cn_da": Cn_da,
"Cn_Lda": Cn_Lda,
"Cn_dr": Cn_dr}}
return coeff_database
c_w = 11.46
b_w = 31.92
V = 222.5211
if __name__ == "__main__":
plt.close(
all
)
nasa = True
save = True
path_to_Ndb_file = 
./nasa_database.csv

path_to_Mdb_file = 
./f16_database.csv

Nfile_exists = exists(path_to_Ndb_file)
Mfile_exists = exists(path_to_Mdb_file)
alpha_range = np.arange(-10., 11., 5.)
N_alpha = len(alpha_range)
beta_range = np.arange(-6., 7., 2.)
N_beta = len(beta_range)
da_range = np.array([-20., 20.])
dr_range = np.array([-30., 30.])
de_range = np.array([-10., 10.])
p_range = np.array([-90., 90.])*np.pi/180.
q_range = np.array([-30., 30.])*np.pi/180.
r_range = np.array([-30., 30.])*np.pi/180.
N_other_a = 1 + len(de_range) + len(p_range) + len(q_range) + len(r_range) +\
len(da_range)
N_other_b = 1 + len(da_range) + len(p_range) + len(r_range) + len(dr_range)
input_file = "F16_input.json"
my_scene = mx.Scene(input_file)
forces_options = {
body_frame
: True,
stab_frame
: False,
wind_frame
: True,
dimensional
: False,
verbose
: False}
if not Nfile_exists:
nasa = True
case = wind.F16_windtunnel()
N_database = np.unique(create_database(), axis=0)
np.savetxt(path_to_Ndb_file, N_database, delimiter=
,
)
if not Mfile_exists:
nasa = False
M_database = np.unique(create_database(), axis=0)
np.savetxt(path_to_Mdb_file, M_database, delimiter=
,
)
if Mfile_exists*Nfile_exists:
M_database = np.genfromtxt(path_to_Mdb_file, delimiter=
,
)
N_database = np.genfromtxt(path_to_Ndb_file, delimiter=
,
)327
if nasa:
N_coeff_data = find_model(N_database)
with open("nasa_model.json", "w") as outfile:
json.dump(N_coeff_data, outfile, indent=4)
M_coeff_data = find_model(M_database)
with open("f16_model.json", "w") as outfile:
json.dump(M_coeff_data, outfile, indent=4)