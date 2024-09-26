import numpy as np
from scipy.interpolate import RegularGridInterpolator as rgi
from scipy.interpolate import interp1d
import stdatmos as atmos

class F16_windtunnel:
    def __init__(self,data_dir="./NASAData/PythonData/"):
        self.c_ref=11.32
        self.b_w=30.
        self.S_w=300.
        self.xcgref_cref=0.35
        alpha = np.concatenate((np.arange(-20.,60.,5.),
                                    np.arange(60.,100.,10.)))
        alpha_lef=np.arange(-20.,50.,5.)
        beta=np.array([-30.,-25.,-20.,-15.,-10.,-8.,-6.,-4.,-2.,0.,
                        2.,4.,6.,8.,10.,15.,20.,25.,30.])
        dh=np.array([-25.,-10.,0.,10.,25.])
        dh_ds=np.array([-25.,-10.,0.,10.,15.,20.,25.])
        dh_n=np.array([-25.,0.,25.])
        M=np.arange(0.0,1.2,0.2)
        H=np.arange(0.0,60000.,10000.)

        # X-force coefficient data import
        C_X=np.load(data_dir +"C_X(a,b,d_h).npy")
        self.CX_abdh=rgi((dh,alpha,beta),C_X,bounds_error=False)
        C_Xlef=np.load(data_dir+"C_X,lef(a,b).npy")
        self.CXlef_ab=rgi((alpha_lef,beta),C_Xlef,bounds_error=False)
        C_Xq=np.load(data_dir+"C_X_q(a).npy")
        self.CXq_a=interp1d(alpha,C_Xq,bounds_error=False,
                            fill_value="extrapolate")
        DC_Xsb=np.load(data_dir+"DC_X,sb(a).npy")
        self.DCXsb_a=interp1d(alpha,DC_Xsb,bounds_error=False,
                                fill_value="extrapolate")
        DC_Xqlef=np.load(data_dir+"DC_X_q,lef(a).npy")
        self.DCXqlef_a=interp1d(alpha_lef,DC_Xqlef,bounds_error=False,
                                fill_value="extrapolate")

        # Y-force coefficient data import
        C_Y=np.load(data_dir+"C_Y(a,b).npy")
        self.CY_ab=rgi((alpha,beta),C_Y,bounds_error=False)
        C_Yda20=np.load(data_dir+"C_Y,d_a=20(a,b).npy")
        self.CYda20_ab=rgi((alpha,beta),C_Yda20,bounds_error=False)
        C_Yda20lef=np.load(data_dir+"C_Y,d_a=20,lef(a,b).npy")
        self.CYda20lef_ab =rgi((alpha_lef,beta),C_Yda20lef,bounds_error=False)
        C_Ydr30=np.load(data_dir+"C_Y,d_r=30(a,b).npy")
        self.CYdr30_ab=rgi((alpha,beta),C_Ydr30,bounds_error=False)
        C_Ylef=np.load(data_dir+"C_Y,lef(a,b).npy")
        self.CYlef_ab=rgi((alpha_lef,beta),C_Ylef,bounds_error=False)
        C_Yp=np.load(data_dir+"C_Y_p(a).npy")
        self.CYp_a=interp1d(alpha,C_Yp,bounds_error=False,
                            fill_value="extrapolate")
        C_Yr=np.load(data_dir+"C_Y_r(a).npy")
        self.CYr_a=interp1d(alpha,C_Yr,bounds_error=False,
                            fill_value="extrapolate")
        DC_Yplef=np.load(data_dir+"DC_Y_p,lef(a).npy")
        self.DCYplef_a=interp1d(alpha_lef,DC_Yplef,bounds_error=False,
                                fill_value="extrapolate")
        DC_Yrlef=np.load(data_dir+"DC_Y_r,lef(a).npy")
        self.DCYrlef_a=interp1d(alpha_lef,DC_Yrlef,bounds_error=False,
                                fill_value="extrapolate")

        # Z-force coefficient data import
        C_Z=np.load(data_dir+"C_Z(a,b,d_h).npy")
        self.CZ_abdh=rgi((dh,alpha,beta),C_Z,bounds_error=False)
        C_Zlef=np.load(data_dir+"C_Z,lef(a,b).npy")
        self.CZlef_ab=rgi((alpha_lef,beta),C_Zlef,bounds_error=False)
        C_Zq=np.load(data_dir+"C_Z_q(a).npy")
        self.CZq_a=interp1d(alpha,C_Zq,bounds_error=False,
                            fill_value="extrapolate")
        DC_Zsb=np.load(data_dir+"DC_Z,sb(a).npy")
        self.DCZsb_a=interp1d(alpha,DC_Zsb,bounds_error=False,
                                fill_value="extrapolate")
        DC_Zqlef=np.load(data_dir+"DC_Z_q,lef(a).npy")
        self.DCZqlef_a=interp1d(alpha_lef,DC_Zqlef,bounds_error=False,
                                fill_value="extrapolate")

        # Pitching moment coefficient data import
        C_m=np.load(data_dir+"C_m(a,b,d_h).npy")
        self.Cm_abdh=rgi((dh,alpha,beta),C_m,bounds_error=False)
        C_mlef=np.load(data_dir+"C_m,lef(a,b).npy")
        self.Cmlef_ab=rgi((alpha_lef,beta),C_mlef,bounds_error=False)
        C_mq=np.load(data_dir+"C_m_q(a).npy")
        self.Cmq_a=interp1d(alpha,C_mq,bounds_error=False,
                            fill_value="extrapolate")
        DC_m=np.load(data_dir+"DC_m(a).npy")
        self.DCm_a=interp1d(alpha,DC_m,bounds_error=False,
                            fill_value="extrapolate")
        DC_mds=np.load(data_dir+"DC_m,ds(a,d_h).npy")
        self.DCmds_adh=rgi((alpha,dh_ds),DC_mds,bounds_error=False)
        DC_msb=np.load(data_dir+"DC_m,sb(a).npy")
        self.DCmsb_a=interp1d(alpha,DC_msb,bounds_error=False,
                                fill_value="extrapolate")
        DC_mqlef=np.load(data_dir+"DC_m_q,lef(a).npy")
        self.DCmqlef_a=interp1d(alpha_lef,DC_mqlef,bounds_error=False,
                                fill_value="extrapolate")
        n_dh=np.load(data_dir+"n_d_h(d_h).npy")
        self.ndh_dh=interp1d(dh,n_dh,bounds_error=False,
                            fill_value="extrapolate")

        # Rolling moment coefficient data import
        C_l=np.load(data_dir+"C_l(a,b,d_h).npy")
        self.Cl_abdh=rgi((dh_n,alpha,beta),C_l,bounds_error=False)
        C_lda20=np.load(data_dir+"C_l,d_a=20(a,b).npy")
        self.Clda20_ab=rgi((alpha,beta),C_lda20,bounds_error=False)
        C_lda20lef=np.load(data_dir+"C_l,d_a=20,lef(a,b).npy")
        self.Clda20lef_ab =rgi((alpha_lef,beta),C_lda20lef,bounds_error=False)
        C_ldr30=np.load(data_dir+"C_l,d_r=30(a,b).npy")
        self.Cldr30_ab=rgi((alpha,beta),C_ldr30,bounds_error=False)
        C_llef=np.load(data_dir+"C_l,lef(a,b).npy")
        self.Cllef_ab=rgi((alpha_lef,beta),C_llef,bounds_error=False)
        C_lp=np.load(data_dir+"C_l_p(a).npy")
        self.Clp_a=interp1d(alpha,C_lp,bounds_error=False,
                            fill_value="extrapolate")
        C_lr=np.load(data_dir+"C_l_r(a).npy")
        self.Clr_a=interp1d(alpha,C_lr,bounds_error=False,
                            fill_value="extrapolate")
        DC_lb=np.load(data_dir+"DC_l_b(a).npy")
        self.DClb_a=interp1d(alpha,DC_lb,bounds_error=False,
                            fill_value="extrapolate")
        DC_lplef=np.load(data_dir+"DC_l_p,lef(a).npy")
        self.DClplef_a=interp1d(alpha_lef,DC_lplef,bounds_error=False,
                                fill_value="extrapolate")
        DC_lrlef=np.load(data_dir+"DC_l_r,lef(a).npy")
        self.DClrlef_a=interp1d(alpha_lef,DC_lrlef,bounds_error=False,
                                fill_value="extrapolate")

        # Yawing moment coefficient data import
        C_n=np.load(data_dir+"C_n(a,b,d_h).npy")
        self.Cn_abdh=rgi((dh_n,alpha,beta),C_n,bounds_error=False)
        C_nda20=np.load(data_dir+"C_n,d_a=20(a,b).npy")
        self.Cnda20_ab=rgi((alpha,beta),C_nda20,bounds_error=False)
        C_nda20lef=np.load(data_dir+"C_n,d_a=20,lef(a,b).npy")
        self.Cnda20lef_ab =rgi((alpha_lef,beta),C_nda20lef,bounds_error=False)
        C_ndr30=np.load(data_dir+"C_n,d_r=30(a,b).npy")
        self.Cndr30_ab=rgi((alpha,beta),C_ndr30,bounds_error=False)
        C_nlef=np.load(data_dir+"C_n,lef(a,b).npy")
        self.Cnlef_ab=rgi((alpha_lef,beta),C_nlef,bounds_error=False)
        C_np=np.load(data_dir+"C_n_p(a).npy")
        self.Cnp_a=interp1d(alpha,C_np,bounds_error=False,
                            fill_value="extrapolate")
        C_nr=np.load(data_dir+"C_n_r(a).npy")
        self.Cnr_a=interp1d(alpha,C_nr,bounds_error=False,
                            fill_value="extrapolate")
        DC_nb=np.load(data_dir+"DC_n_b(a).npy")
        self.DCnb_a=interp1d(alpha,DC_nb,bounds_error=False,
                            fill_value="extrapolate")
        DC_nda=np.load(data_dir+"DC_n_d_a(a).npy")
        self.DCnda_a=interp1d(alpha,DC_nda,bounds_error=False,
                                fill_value="extrapolate")
        DC_nplef=np.load(data_dir+"DC_n_p,lef(a).npy")
        self.DCnplef_a=interp1d(alpha_lef,DC_nplef,bounds_error=False,
                                fill_value="extrapolate")
        DC_nrlef=np.load(data_dir+"DC_n_r,lef(a).npy")
        self.DCnrlef_a=interp1d(alpha_lef,DC_nrlef,bounds_error=False,
                                fill_value="extrapolate")

        T=np.load(data_dir+"Thrust_En(M,H).npy")
        self.T_idle=rgi((M,H),T[0])
        self.T_mil=rgi((M,H),T[1])
        self.T_max=rgi((M,H),T[2])


    def set_state(self,alpha,beta,dh,dlef,dsb,da,dr,
                    xcg_cref,p,q,r,M,H,thtl):
        self.alpha=alpha
        self.beta=beta
        self.dh=dh
        self.dlef=dlef
        self.dsb=dsb
        self.da=da
        self.dr=dr
        self.xcg_cref=xcg_cref
        self.p=p
        self.q=q
        self.r=r
        self.H=H
        self.rho,self.a=atmos.stdatm_english(H)[-2:]
        self.M=M
        self.V=M*self.a
        self.pbar=p*self.b_w/(2.*self.V)
        self.qbar=q*self.c_ref/(2.*self.V)
        self.rbar=r*self.b_w/(2.*self.V)
        self.thtl=thtl

    def _CX(self):
        DC_Xlef=self.CXlef_ab([self.alpha,self.beta])-\
                self.CX_abdh([0.,self.alpha,self.beta])
        C_Xt = self.CX_abdh([self.dh,self.alpha,self.beta])+\
               DC_Xlef*(1.-(self.dlef/25.))+\
               self.DCXsb_a(self.alpha)*(self.dsb/60.)+\
               self.qbar*(self.CXq_a(self.alpha)+
               self.DCXqlef_a(self.alpha)*(1.-(self.dlef/25.)))
        return C_Xt[0]

    def _CZ(self):
        DC_Zlef=self.CZlef_ab([self.alpha,self.beta])-\
                self.CZ_abdh([0.,self.alpha,self.beta])
        C_Zt=self.CZ_abdh([self.dh,self.alpha,self.beta])+\
            DC_Zlef*(1.-(self.dlef/25.))+\
            self.DCZsb_a(self.alpha)*(self.dsb/60.)+\
            self.qbar*(self.CZq_a(self.alpha)+
            self.DCZqlef_a(self.alpha)*(1.-(self.dlef/25.)))
        return C_Zt[0]

    def _Cm(self):
        DC_mlef=self.Cmlef_ab([self.alpha,self.beta])-\
                self.Cm_abdh([0.,self.alpha,self.beta])
        C_mt=self.Cm_abdh([self.dh,self.alpha,self.beta])*self.ndh_dh(self.dh)+\
            self.C_Zt*(self.xcgref_cref-self.xcg_cref)+\
            DC_mlef*(1.-(self.dlef/25.))+\
            self.DCmsb_a(self.alpha)*(self.dsb/60.)+\
            self.qbar*(self.Cmq_a(self.alpha)+
                        self.DCmqlef_a(self.alpha)*(1.-(self.dlef/25.)))+\
            self.DCm_a(self.alpha)+\
            self.DCmds_adh([self.alpha,self.dh])
        return C_mt[0]

    def _CY(self):
        ab=[self.alpha, self.beta]
        DC_Ylef=self.CYlef_ab(ab)-self.CY_ab(ab)
        DC_Yda20=self.CYda20_ab(ab)-self.CY_ab(ab)
        DC_Yda20lef=self.CYda20lef_ab(ab)-self.CYlef_ab(ab)-DC_Yda20
        DC_Ydr30=self.CYdr30_ab(ab)-self.CY_ab(ab)
        C_Yt=self.CY_ab(ab)+\
            DC_Ylef*(1.-(self.dlef/25.))+\
            (DC_Yda20+DC_Yda20lef*(1.-(self.dlef/25.)))*(self.da/20.)+\
            DC_Ydr30*(self.dr/30.)+\
            self.rbar*(self.CYr_a(self.alpha)+
                        self.DCYrlef_a(self.alpha)*(1.-(self.dlef/25.)))+\
            self.pbar*(self.CYp_a(self.alpha)+
                        self.DCYplef_a(self.alpha)*(1.-(self.dlef/25.)))
        return C_Yt[0]

    def _Cn(self):
        ab=[self.alpha, self.beta]
        Cn_abdh0=self.Cn_abdh([0.,self.alpha,self.beta])
        DC_nlef=self.Cnlef_ab(ab)-Cn_abdh0
        DC_nda20=self.Cnda20_ab(ab)-Cn_abdh0
        DC_nda20lef=self.Cnda20lef_ab(ab)-self.Cnlef_ab(ab)-DC_nda20
        DC_ndr30=self.Cndr30_ab(ab)-Cn_abdh0
        C_nt=self.Cn_abdh([self.dh,self.alpha,self.beta])+\
            DC_nlef*(1.-(self.dlef/25.))-\
            self.C_Yt*(self.xcgref_cref-self.xcg_cref)*(self.c_ref/self.b_w)+\
            (DC_nda20+DC_nda20lef*(1.-(self.dlef/25.)))*(self.da/20.)+\
            DC_ndr30*(self.dr/30.)+\
            self.rbar*(self.Cnr_a(self.alpha)+
                        self.DCnrlef_a(self.alpha)*(1.-(self.dlef/25.)))+\
            self.pbar*(self.Cnp_a(self.alpha)+
                        self.DCnplef_a(self.alpha)*(1.-(self.dlef/25.)))+\
            self.DCnb_a(self.alpha)*np.deg2rad(self.beta)
        return C_nt[0]

    def _Cl(self):
        ab=[self.alpha, self.beta]
        Cl_abdh0=self.Cl_abdh([0.,self.alpha,self.beta])
        DC_llef=self.Cllef_ab(ab)-Cl_abdh0
        DC_lda20=self.Clda20_ab(ab)-Cl_abdh0
        DC_lda20lef=self.Clda20lef_ab(ab)-self.Cllef_ab(ab)-DC_lda20
        DC_ldr30=self.Cldr30_ab(ab)-Cl_abdh0
        C_lt=self.Cl_abdh([self.dh,self.alpha,self.beta])+\
            DC_llef*(1.-(self.dlef/25.))+\
            (DC_lda20+DC_lda20lef*(1.-(self.dlef/25.)))*(self.da/20.)+\
            DC_ldr30*(self.dr/30.)+\
            self.pbar*(self.Clp_a(self.alpha)+
                        self.DClplef_a(self.alpha)*(1.-(self.dlef/25.)))+\
            self.rbar*(self.Clr_a(self.alpha)+
                        self.DClrlef_a(self.alpha)*(1.-(self.dlef/25.)))+\
            self.DClb_a(self.alpha)*np.deg2rad(self.beta)
        return C_lt[0]

    def _tau_inv(self,dp):
        if dp<=25.0:
            t_inv=1.0
        elif dp>=50.0:
            t_inv=0.1
        else:
            t_inv=1.9- 0.036*dp
        return t_inv

    def thrust(self):
        MH=[self.M,self.H]
        if self.thtl<=0.77:
            pow_c=64.94*self.thtl
        else:
            pow_c=217.38*self.thtl-117.38
        if pow_c>=50.0:
            T=self.T_mil(MH) + (self.T_max(MH) - 
                                self.T_mil(MH))*(pow_c-50.0)/50.0
        else:
            T=self.T_idle(MH)+(self.T_mil(MH)-self.T_idle(MH))*pow_c/50.0
        return T

    def _body_to_stab(self,FM,alpha):
        CX,CY,CZ,Cl,Cm,Cn=FM
        CA=-CX
        CN=-CZ
        alpha_rad=np.deg2rad(alpha)
        c_a=np.cos(alpha_rad)
        s_a=np.sin(alpha_rad)
        CD_s=CA*c_a+CN*s_a
        CL=CN*c_a-CA*s_a
        Cl_s=Cl*c_a+Cn*s_a
        Cn_s=Cn*c_a-Cl*s_a
        return [CD_s,CY, CL,Cl_s,Cm,Cn_s]

    def _body_to_wind(self,FM,alpha,beta):
        CX,CY,CZ,Cl,Cm,Cn=FM
        CA=-CX
        CN=-CZ
        alpha_rad=np.deg2rad(alpha)
        beta_rad=np.deg2rad(beta)
        c_a=np.cos(alpha_rad)
        s_a=np.sin(alpha_rad)
        c_b=np.cos(beta_rad)
        s_b=np.sin(beta_rad)
        CD=CA*c_a*c_b- CY*s_b+CN*s_a*c_b
        CS=CA*c_a*s_b+ CY*c_b+CN*s_a*s_b
        CL=CN*c_a-CA*s_a
        Cl_w=Cl*c_a*c_b +Cm*s_b+Cn*s_a*c_b
        Cm_w=Cm*c_b-Cl*c_a*s_b-Cn*s_a*s_b
        Cn_w=Cn*c_a-Cl*s_a
        return [CD,CS,CL,Cl_w,Cm_w,Cn_w]


    def calc_forces(self, body_frame=True,stab_frame=False,
                    wind_frame=False,dimensional=False,
                    verbose=False):
        self.C_Xt=self._CX()
        self.C_Zt=self._CZ()
        self.C_mt=self._Cm()
        self.C_Yt=self._CY()
        self.C_nt=self._Cn()
        self.C_lt=self._Cl()
        body_fm=[self.C_Xt,self.C_Yt,self.C_Zt,
                self.C_lt,self.C_mt,self.C_nt]
        dim_const = 0.5*self.rho*self.V**2*self.S_w
        forces={'F16' : {}}
        if body_frame:
            body_keys=[ 'CX', 'CY', 'CZ', 'Cl', 'Cm', 'Cn']
            forces['F16'].update({key : fm for key, fm in zip(body_keys,body_fm)})
            if dimensional:
                body_keys_dim=['Fx_b', 'Fy_b', 'Fz_b', 'Mx_b', 'My_b', 'Mz_b']
                body_fm_dim=[fm*dim_const for fm in body_fm]
                body_fm_dim[3]*=self.b_w
                body_fm_dim[4]*=self.c_ref
                body_fm_dim[5]*=self.b_w
                forces['F16'].update({key:fm for key, fm in zip(body_keys_dim,
                                                                body_fm_dim)})
        if stab_frame:
            stab_keys=[ 'CD_s', 'CY_s', 'CL_s', 'Cl_s', 'Cm_s', 'Cn_s']
            stab_fm=self._body_to_stab(body_fm,self.alpha)
            forces['F16'].update({key:fm for key,fm in zip(stab_keys,stab_fm)})
            if dimensional:
                stab_keys_dim=['Fx_s', 'Fy_s', 'Fz_s', 'Mx_s', 'My_s', 'Mz_s']
                stab_fm_dim=[fm*dim_const for fm in stab_fm]
                stab_fm_dim[3]*=self.b_w
                stab_fm_dim[4]*=self.c_ref
                stab_fm_dim[5]*=self.b_w
                forces['F16'].update({key : fm for key, fm in zip(stab_keys_dim,
                                                                stab_fm_dim)})
        if wind_frame:
            wind_keys=[ 'CD', 'CS', 'CL', 'Cl_w', 'Cm_w', 'Cn_w']
            wind_fm=self._body_to_wind(body_fm,self.alpha,self.beta)
            forces['F16'].update({key:fm for key,fm in zip(wind_keys,wind_fm)})
            if dimensional:
                wind_keys_dim=['Fx_w', 'Fy_w', 'Fz_w', 'Mx_w', 'My_w', 'Mz_w']
                wind_fm_dim=[fm*dim_const for fm in wind_fm]
                wind_fm_dim[3]*=self.b_w
                wind_fm_dim[4]*=self.c_ref
                wind_fm_dim[5]*=self.b_w
                forces['F16'].update({key:fm for key, fm in zip(wind_keys_dim,
                                                                wind_fm_dim)})
        if verbose:
            print(forces['F16'])
        return forces

if __name__=="__main__":
    case = F16_windtunnel()
    alpha=0.
    beta=0.
    dh=10.
    da=0.
    dr=0.
    p=0.
    q=0.
    r=0.
    xcg_cref = 0.
    dsb = 0.
    dlef = 0.
    H = 1000.
    M = 0.2
    thtl = 0.
    case.set_state(alpha, beta, dh, dlef, dsb, da, dr, xcg_cref, p, q, r, M, H, thtl)
    forces_options = {'body_frame': True,
                       ' stab_frame': True,
                        'wind_frame': True,
                        'dimensional': True,
                        'verbose': True}
    fm = case.calc_forces(**forces_options)