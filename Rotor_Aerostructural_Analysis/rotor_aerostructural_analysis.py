# rotor_design_test.py
# 
# Created:  Sep 2014, E. Botero
# Modified: Feb 2020, M. Clarke  
#           Sep 2020, M. Clarke 
#           Nov 2022, M. Clarke 

#----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Units
from SUAVE.Plots.Geometry import plot_rotor
import matplotlib.pyplot as plt  
from SUAVE.Core import Data 

from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_properties \
     import compute_airfoil_properties
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_naca_4series \
     import compute_naca_4series
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry\
     import import_airfoil_geometry

import numpy as np
import scipy as sp
import copy 
from SUAVE.Methods.Propulsion import propeller_design , lift_rotor_design , prop_rotor_design  

def main():

    # This script could fail if either the design or analysis scripts fail,
    # in case of failure check both. The design and analysis powers will 
    # differ because of karman-tsien compressibility corrections in the 
    
    # blade properties  density 
    rho_blade  = 2810      # blade material density 
    G          = 26.9*10E9  # shear modulus 
    Y          = 503 *10E6  # yeild    
    g          = 9.81 
    beta_e_d   = 1*Units.degree # chosen by user # max aeroelastic twist
    sigma_d    = Y/2 # chosen by user # max design normal stress 
    tau_d      = Y/4 # chosen by user # max design shear stress  
    
    # data dimension = [ num cpts, num stations, num surface points ]
    # analysis scripts  
    rotor,F,Q,P,Cp,performance_data,etap = lift_rotor_run()
    
    num_cpt   = len(F)  
    num_sec   = len(rotor.chord_distribution)  
    airfoils  = rotor.Airfoils
    num_pts   = rotor.Airfoils[airfoils.keys()[0]].number_of_points 
    
    c        = np.tile(rotor.chord_distribution[None,:,None],(num_cpt,1,num_pts)) 
    r        = np.tile(rotor.radius_distribution[None,:],(num_cpt,1)) 
    R        = rotor.tip_radius # blade radius  
    theta    = np.zeros_like(r) # rake/dihedral of blade  
    omega    = np.tile(performance_data.omega,(1,num_sec)) 
    rho      = np.tile(performance_data.density,(1,num_sec))   
    V_b      = omega*r                        # local total velocity at blade section
    V_t      = omega*rotor.tip_radius        # blade tip speed of blade 
    C_fx     = performance_data.lift_coefficient  #x force coeff
    C_fy     = performance_data.drag_coefficient  # y force coeff 
    
    # quarter chord properties 
    x_qc     = c[:,:,0]/4
    y_qc     = np.zeros_like(x_qc) # assumed to be on the chord line 
    C_m_qc   = performance_data.moment_coefficient # need to change  
    
    x0, y0 ,r_c,t,u = compute_dimensional_blade_properties(rotor)
    
    x0  = np.tile(x0[None,:,:],(num_cpt,1,1))
    y0  = np.tile(y0[None,:,:],(num_cpt,1,1)) 
    r_c = np.tile(r_c[None,:],(num_cpt,1))
    t   = np.tile(t[None,:,:],(num_cpt,1,1))  
    u   = np.tile(u[None,:,:],(num_cpt,1,1))  
    
    x_pts        = x0[:,:,::-1]  
    y_pts        = y0[:,:,::-1]   
    
    # area 
    a = x_pts[:,:,:-1]*y_pts[:,:,1:] -  x_pts[:,:,1:]*y_pts[:,:,:-1]
    A = 0.5*np.sum(a,axis = 2)
    
    # centroid 
    x_c   = 1/(A*6) *np.sum((x_pts[:,:,:-1] + x_pts[:,:,1:])*a, axis = 2)
    y_c   = 1/(A*6) *np.sum((y_pts[:,:,:-1] + y_pts[:,:,1:])*a, axis = 2)
    
    # center of gravity (colocated with centroid)
    x_cg  = x_c 
    
    # shear center  (colocated with centroid)
    x_sc   = x_c
    y_sc   = y_c 
    
    x_p = x_pts - np.tile(x_c[:,:,None],(1,1,num_pts))
    y_p = y_pts - np.tile(y_c[:,:,None],(1,1,num_pts))
    a_p = x_p[:,:,:-1]*y_p[:,:,1:] -  x_p[:,:,1:]*y_p[:,:,:-1]
    
    # section intertial properties 
    I_x  = (1/12)*np.sum((y_p[:,:,:-1]**2 + y_p[:,:,:-1]*y_p[:,:,1:] +  y_p[:,:,1:]**2  )*a_p, axis = 2)
    I_y  = (1/12)*np.sum((x_p[:,:,:-1]**2 + x_p[:,:,:-1]*x_p[:,:,1:] +  x_p[:,:,1:]**2  )*a_p, axis = 2)
    I_xy = (1/24)*np.sum((x_p[:,:,:-1]*y_p[:,:,1:]  + 2*x_p[:,:,:-1]*y_p[:,:,:-1] +  x_p[:,:,1:]*y_p[:,:,:-1])*a_p, axis = 2)
      
    # torsion constant K for a solid homogeneous section (ref 28) page 406 
    F = np.trapz(t**3, x = u, axis = 2 ) 
    K = F/(3+ (4*F/A*u[:,:,-1]**2)) 
    
    # cumulative centrifugal force 
    F_cz   = np.zeros((num_cpt,num_sec))
    F_cz_2  = np.zeros((num_cpt,num_sec))
    for sec in range(num_sec):
        F_cz[:,sec]   = rho_blade*(V_t[:,sec]**2)* np.trapz(A[:,sec:]*r[:,sec]/rotor.tip_radius, x = r[:,sec:], axis =1) #CHECK 

        F_cz_2[:,sec] = rho_blade* (omega[:,sec]**2)  * np.trapz(A[:,sec:]* r[:,sec] , x = r[:,sec:], axis =1)        #CHECK    
    
    # distributed mass of blade (centered at outer station of each segment)
    m_blade    = np.zeros((num_cpt,num_sec)) 
    m_blade[:,1:]  = (0.5*(A[:,:-1]+A[:,1:])*np.diff(r[0]))*rho_blade
    
    q_cy = rho_blade*(V_t**2)*A*np.sin(theta)  #CHECK 
    q_cy_2 = (m_blade*(omega**2)*r)*  np.sin(theta)  #CHECK 
    
    q_x  = 0.5*rho*(V_b**2)*c[:,:,0]*C_fx
    q_y  = 0.5*rho*(V_b**2)*c[:,:,0]*C_fy - q_cy
    

    S_x   = np.zeros((num_cpt,num_sec)) 
    S_y   = np.zeros((num_cpt,num_sec))   
    for sec in range(num_sec):
        # shear forces 
        S_x[:,sec] = -np.trapz(q_x[:,sec:], x = r[:,sec:],axis = 1 )
        S_y[:,sec] = -np.trapz(q_y[:,sec:], x = r[:,sec:],axis = 1 )    
  
    M_x   = np.zeros((num_cpt,num_sec)) 
    M_y   = np.zeros((num_cpt,num_sec))    
    for sec in range(num_sec): 
        # bending moment
        M_x[:,sec] = - np.trapz(S_x[:,sec:], x = r[:,sec:],axis = 1 )
        M_y[:,sec] = - np.trapz(S_y[:,sec:], x = r[:,sec:],axis = 1 )
     
    
    # normal stress at a given point i on the section 
    M1  = np.tile((M_y*I_x + M_x*I_xy)[:,:,None],(1,1,num_cpt))
    M2  = np.tile((M_x*I_y + M_y*I_xy)[:,:,None],(1,1,num_cpt))
    I1  = np.tile((I_x*I_y - I_xy**2)[:,:,None],(1,1,num_cpt))
    sigma_n_i = (-M1*x_pts + M2*y_pts)/I1
    
    # centrifugal normal stress 
    sigma_c = F_cz/A
    
    # maxiumum normal stress due to mending in the section (of the blade) 
    sigma_z = np.max(np.tile(sigma_c[:,:,None],(1,1,num_pts)) - sigma_n_i, axis = 2)
    
    # compute weight of blade 
    F_cy         = np.zeros_like(A)
    F_cy[:,1:]   = rho_blade*(A[:,1:]*np.diff(r))*g
    
    # applied torque at radius r i.e. M_t 
    # subscript sc denotes the shear center 
    #location. The shear center is assumed to be collocated with the center of gravity at the geometric centroid
    
    M_t0     = 0.5*rho*(V_b**2)*(c[:,:,0]**2)
    C_m_f_x  = C_fx*(-(y_sc - y_qc)/c[:,:,0])
    C_m_f_y  = C_fx*((x_sc - x_qc)/c[:,:,0]) 
    C_m_c    = F_cy*(-(x_sc - x_cg)/M_t0) 

    M_t    = np.zeros((num_cpt,num_sec))    
    for sec in range(num_sec): 
        # bending moment
        M_val      = M_t0*(C_m_f_x +C_m_f_y+ C_m_qc + C_m_c ) 
        M_t[:,sec] = - np.trapz(M_val[:,sec:], x = r[:,sec:],axis = 1 ) 
    
    # the elastic twist   
    beta_e  = np.zeros((num_cpt,num_sec))    
    for sec in range(num_sec):  
        beta_e_r      = M_t/(K*G)
        beta_e[:,sec] = np.trapz(beta_e_r[:,:sec], x = r[:,:sec], axis = 1)
    
    # max shear stress
    D      = np.max(t) 
    C      = (D/(1 + (((np.pi**2)*(D**4) )/(16*A**2))))*( 1 + 0.15*( (((np.pi**2)*(D**4) )/(16*A**2))- ( D/(2*r_c)))) 
    tau_xy = M_t*C/K
    taus    = np.sort(np.vstack((np.vstack((tau_xy,-tau_xy)),sigma_z)),axis = 0)
    sigma_1 = taus[2,:]
    sigma_2 = taus[1,:]
    sigma_3 = taus[0,:]
    
    # define max stresses
    tau_max   = np.linalg.norm(sigma_1 -sigma_3)/2
    sigma_max = np.max(taus)
    
    failure_flag_1 = abs(sigma_max)<sigma_d
    failure_flag_2 = abs(tau_max)<tau_d
    failure_flag_3 = max(beta_e)<beta_e_d
    
    
    volume     = np.trapz(A,x=r,axis=1) 
    blade_weight = (volume*rho_blade)*g
    
    return failure_flag_1,failure_flag_2,failure_flag_3,blade_weight

def compute_dimensional_blade_properties(rotor):
    '''
    assumes the total number of points each airfoil section of the blade are equal'''
    
    
    airfoils      = rotor.Airfoils
    a_loc         = rotor.airfoil_polar_stations
    c             = rotor.chord_distribution  
    
    num_pts       = rotor.Airfoils[airfoils.keys()[0]].number_of_points
    hnum_pts      = len(rotor.Airfoils[airfoils.keys()[0]].geometry.x_upper_surface)
    n_sec         = len(c)
    x             = np.zeros((n_sec,num_pts)) 
    y             = np.zeros_like(x) 
    x_us          = np.zeros((n_sec,hnum_pts))    
    y_us          = np.zeros_like(x_us) 
    x_ls          = np.zeros_like(x_us)     
    y_ls          = np.zeros_like(x_us) 
    cam_y         = np.zeros_like(x_us)
    U             = np.zeros_like(x_us)
    thickness     = np.zeros_like(x_us)  
    r_c           = np.zeros(n_sec)
    mt_loc        = np.zeros(n_sec)
    
    if len(airfoils.keys())>0:
        for j,airfoil in enumerate(airfoils): 
            a_geo           = airfoil.geometry
            locs            = np.where(np.array(a_loc) == j )[0]
            chord           = np.tile(c[:,None],(1,num_pts)) 
            hchord          = np.tile(c[:,None],(1,hnum_pts))  
            x_pts           = np.tile(a_geo.x_coordinates[None,:],(len(locs),1))*chord[locs]       
            y_pts           = np.tile(a_geo.y_coordinates[None,:],(len(locs),1))*chord[locs]
            x_upper_surf    = np.tile(a_geo.x_upper_surface[None,:],(len(locs),1))*hchord[locs]
            y_upper_surf    = np.tile(a_geo.y_upper_surface[None,:],(len(locs),1))*hchord[locs] 
            x_lower_surf    = np.tile(a_geo.x_lower_surface[None,:],(len(locs),1))*hchord[locs]
            y_lower_surf    = np.tile(a_geo.y_lower_surface[None,:],(len(locs),1))*hchord[locs] 
            camber          = np.tile(a_geo.camber_coordinates[None,:],(len(locs),1))*hchord[locs] 
            x[locs]         = x_pts   
            y[locs]         = y_pts
            x_us[locs]      = x_upper_surf
            y_us[locs]      = y_upper_surf
            x_ls[locs]      = x_lower_surf
            y_ls[locs]      = y_lower_surf
            cam_y[locs]     = camber
            U[locs,1:]      = np.cumsum(np.sqrt((x_upper_surf[:,1:] - x_upper_surf[:,:-1])**2 +  (camber[:,1:]- camber[:,:-1])**2), axis = 1) 
            thickness[locs] = y_upper_surf[locs] - y_lower_surf[locs]
            mt_loc          = np.argmax(y_upper_surf[locs] - y_lower_surf[locs], axis = 1)  
            r_c_us          = radius_of_curvature(x_upper_surf[:,mt_loc-1][:,0],y_upper_surf[:,mt_loc-1][:,0],
                                                  x_upper_surf[:,mt_loc][:,0]  ,y_upper_surf[:,mt_loc][:,0],
                                                  x_upper_surf[:,mt_loc+1][:,0],y_upper_surf[:,mt_loc+1][:,0])
            r_c_ls          = radius_of_curvature(x_lower_surf[:,mt_loc-1][:,0],y_lower_surf[:,mt_loc-1][:,0],
                                                  x_lower_surf[:,mt_loc][:,0]  ,y_lower_surf[:,mt_loc][:,0],
                                                  x_lower_surf[:,mt_loc+1][:,0],y_lower_surf[:,mt_loc+1][:,0])
            r_c[locs]       = np.maximum(r_c_us,r_c_ls)
            
    return x, y ,r_c,thickness,U
 
def radius_of_curvature(x1,y1,x2,y2,x3,y3): 
    c  = (x1-x2)**2 + (y1-y2)**2
    a  = (x2-x3)**2 + (y2-y3)**2
    b  = (x3-x1)**2 + (y3-y1)**2 
    ar = a**0.5
    br = b**0.5
    cr = c**0.5 
    r  = ar*br*cr / ((ar+br+cr)*(-ar+br+cr)*(ar-br+cr)*(ar+br-cr))**0.5    
    return r
 
def lift_rotor_run():     

    rotor                              = SUAVE.Components.Energy.Converters.Lift_Rotor() 
    rotor.tag                          = 'rotor'
    rotor.orientation_euler_angles     = [0, 90*Units.degrees,0]
    rotor.tip_radius                   = 1.15
    rotor.hub_radius                   = 0.15 * rotor.tip_radius  
    rotor.number_of_blades             = 3 
    rotor.design_tip_mach              = 0.65   
    rotor.design_Cl                    = 0.7
    rotor.design_altitude              = 40 * Units.feet  
    rotor.design_thrust                = 2000
    rotor.angular_velocity             = rotor.design_tip_mach* 343 /rotor.tip_radius  
    rotor.freestream_velocity          = np.sqrt(rotor.design_thrust/(2*1.2*np.pi*(rotor.tip_radius**2))) 
    

    # define first airfoil 
    airfoil                         = SUAVE.Components.Airfoils.Airfoil()
    airfoil.tag                     = 'NACA_4412' 
    airfoil.coordinate_file         = 'Airfoils/NACA_4412.txt'   # absolute path   
    airfoil.polar_files             = ['Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                         'Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                            'Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                            'Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                            'Airfoils/Polars/NACA_4412_polar_Re_1000000.txt'] 
    
    rotor.airfoil_polar_stations   =  [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    rotor.append_airfoil(airfoil)   
    
    rotor.variable_pitch               = True       
    opt_params                         = rotor.optimization_parameters 
    opt_params.aeroacoustic_weight     = 1.0 # 1 means only perfomrance optimization 0.5 to weight noise equally  
    rotor                              = lift_rotor_design(rotor)  # Reduced iteration for regression therefore optimal design is NOT reached!   
    
    # Find the operating conditions
    atmosphere                                          = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere_conditions                               =  atmosphere.compute_values(rotor.design_altitude)  
    conditions                                          = SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics()
    conditions._size                                    = 1
    conditions.freestream                               = Data()
    conditions.propulsion                               = Data()
    conditions.frames                                   = Data()
    conditions.frames.body                              = Data()
    conditions.frames.inertial                          = Data()
    conditions.freestream.update(atmosphere_conditions) 
    conditions.frames.inertial.velocity_vector          = np.array([[0, 0. ,rotor.freestream_velocity]])  
    conditions.propulsion.throttle                      = np.array([[1.0]])
    conditions.frames.body.transform_to_inertial        = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., -1.]]])      

    # Assign rpm
    rotor.inputs.omega  = np.array(rotor.angular_velocity,ndmin=2) 

    # rotor with airfoil results  
    F,Q,P,Cp,output,etap= rotor.spin(conditions) 

    return rotor,F,Q,P,Cp,output,etap

 
# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()
    plt.show()
