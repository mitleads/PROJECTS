# JAX TEXTS

from jax import jit, grad
import jax.numpy as jnp
import numpy as np
import timeit
import time 

'''The follow tests are meant to verify the functionality of JAX in SUAVE'''
 

# ----------------------------------------------------------------------
#   JAX TIME TEST
# ----------------------------------------------------------------------
def numpy_noise_test():
    
    return 


def jax_numpy_noise_test():
    
    return 



# SUAVE Imports 
import SUAVE
from SUAVE.Core import Units, Data 
from SUAVE.Components.Energy.Networks.Battery_Propeller                                   import Battery_Propeller 
from SUAVE.Methods.Noise.Fidelity_One.Propeller.propeller_mid_fidelity                    import propeller_mid_fidelity
from SUAVE.Analyses.Mission.Segments.Conditions                                           import Aerodynamics
from SUAVE.Analyses.Mission.Segments.Segment                                              import Segment 
from SUAVE.Methods.Aerodynamics.Airfoil_Panel_Method.airfoil_analysis      import airfoil_analysis 
import matplotlib.pyplot as plt   
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_naca_4series \
     import  compute_naca_4series 
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools.dbA_noise                    import A_weighting  
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools                              import SPL_harmonic_to_third_octave
from SUAVE.Methods.Noise.Fidelity_One.Propeller.compute_source_coordinates     import compute_point_source_coordinates
from SUAVE.Plots.Geometry import plot_propeller

# Python Imports 
import time 
import numpy as np  
import scipy as sp
from scipy.special import jv 
from scipy.special import fresnel
import matplotlib.pyplot as plt  
import matplotlib.cm as cm 

# import propeller/rotors geometries  
import sys
sys.path.append('../../XX_Supplementary')

from Propellers_Rotors.design_SR2_4_blade_prop import design_SR2_4_blade_prop 
from Propellers_Rotors.design_SR2_8_blade_prop import design_SR2_8_blade_prop
from Propellers_Rotors.design_SR7_8_blade_prop import design_SR7_8_blade_prop 
from Propellers_Rotors.design_BO_105_prop      import design_BO_105_prop 
from Propellers_Rotors.design_Hubbard_prop     import design_Hubbard_prop     
from Propellers_Rotors.design_F8745D4_prop     import design_F8745D4_prop     
from Propellers_Rotors.design_DJI_9_4x5_prop   import design_DJI_9_4x5_prop   
from Propellers_Rotors.design_APC_11x_4_7_prop import design_APC_11x_4_7_prop   
from Propellers_Rotors.design_APC_11x45_prop   import design_APC_11x45_prop   
from Propellers_Rotors.design_APC_10x7_prop    import design_APC_10x7_prop 
from Propellers_Rotors.design_APC_11x8_prop    import design_APC_11x8_prop
from Propellers_Rotors.design_SR1_prop         import design_SR1_prop 
# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main():   
    ti                = time.time()      
 
    Numpy_High_Fidelity_Validation()  

    tf = time.time() 
    print ('time taken: '+ str(round(((tf-ti)),3)) + ' sec')        
    return 
 
def Numpy_High_Fidelity_Validation(PP,save_figures): 

    DJI_CF = design_DJI_9_4x5_prop()
    DJI_CF_inflow_ratio = 0.05 

    # Atmosheric conditions 
    a                     = 343   
    density               = 1.225
    dynamic_viscosity     = 1.78899787e-05   
    T                     = 286.16889478 

    # ---------------------------------------------------------------------------------------------------------------------------
    # APC SF Rotor
    # ---------------------------------------------------------------------------------------------------------------------------
    # Define Network
    net_DJI_CF                                  = Battery_Propeller()
    net_DJI_CF.number_of_propeller_engines      = 1        
    net_DJI_CF.identical_propellers             = True  
    net_DJI_CF.propellers.append(DJI_CF)    

    # Run conditions                            
    DJI_CF_RPM                                  = np.array([6000])
    DJI_CF_omega_vector                         = DJI_CF_RPM * Units.rpm 
    ctrl_pts                                    = len(DJI_CF_omega_vector)   
    velocity                                    = DJI_CF_inflow_ratio*DJI_CF_omega_vector*DJI_CF.tip_radius 
    theta                                       = np.array([120.5])    # random microphones 
    S                                           = 1.51

    # Microphone Locations 
    positions = np.zeros((len(theta),3))
    for i in range(len(theta)):
        if theta[i]*Units.degrees < np.pi/2:
            positions[i][:] = [-S*np.cos(theta[i]*Units.degrees),-S*np.sin(theta[i]*Units.degrees), 0.0]
        else: 
            positions[i][:] = [S*np.sin(theta[i]*Units.degrees- np.pi/2),-S*np.cos(theta[i]*Units.degrees - np.pi/2), 0.0] 

    # Define conditions 
    DJI_CF.thrust_angle                                            = 0. * Units.degrees
    DJI_CF.inputs.omega                                            = np.atleast_2d(DJI_CF_omega_vector).T
    DJI_CF_conditions                                              = Aerodynamics() 
    DJI_CF_conditions.freestream.density                           = np.ones((ctrl_pts,1)) * density
    DJI_CF_conditions.freestream.dynamic_viscosity                 = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    DJI_CF_conditions.freestream.speed_of_sound                    = np.ones((ctrl_pts,1)) * a 
    DJI_CF_conditions.freestream.temperature                       = np.ones((ctrl_pts,1)) * T
    v_mat                                                          = np.zeros((ctrl_pts,3))
    v_mat[:,0]                                                     = velocity 
    DJI_CF_conditions.frames.inertial.velocity_vector              = v_mat 
    DJI_CF_conditions.propulsion.throttle                          = np.ones((ctrl_pts,1)) * 1.0 
    DJI_CF_conditions.frames.body.transform_to_inertial            = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])

    # Run Propeller BEMT new model  
    DJI_CF_thrust, DJI_CF_torque, DJI_CF_power, DJI_CF_Cp, acoustic_outputs  , DJI_CF_etap  =  DJI_CF.spin(DJI_CF_conditions)  

    # Prepare Inputs for Noise Model  
    DJI_CF_conditions.noise.total_microphone_locations             = np.repeat(positions[ np.newaxis,:,: ],ctrl_pts,axis=0)
    DJI_CF_conditions.aerodynamics.angle_of_attack                 = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    DJI_CF_segment                                                 = Segment() 
    DJI_CF_segment.state.conditions                                = DJI_CF_conditions
    DJI_CF_segment.state.conditions.expand_rows(ctrl_pts)
    DJI_CF_settings                                                = Data()
    DJI_CF_settings                                                = setup_noise_settings(DJI_CF_segment)   

    # Run Noise Model    
    DJI_CF_propeller_noise             = propeller_mid_fidelity(net_DJI_CF.propellers,acoustic_outputs,DJI_CF_segment,DJI_CF_settings )     
    
    
    return 
 
def Numpy_setup_noise_settings(sts): 

    sts.ground_microphone_phi_angles   = np.array([30.,45.,60.,75.,89.9,90.1,105.,120.,135.,150.])*Units.degrees
    sts.ground_microphone_theta_angles = np.array([89.9,89.9,89.9,89.9,89.9,89.9,89.9,89.9, 89.9,89.9 ])*Units.degrees
    sts.center_frequencies             = np.array([16,20,25,31.5,40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, \
                                                   500, 630, 800, 1000, 1250, 1600, 2000, 2500, 3150,
                                                   4000, 5000, 6300, 8000, 10000])        
    sts.lower_frequencies              = np.array([14,18,22.4,28,35.5,45,56,71,90,112,140,180,224,280,355,450,560,710,\
                                                   900,1120,1400,1800,2240,2800,3550,4500,5600,7100,9000 ])
    sts.upper_frequencies              = np.array([18,22.4,28,35.5,45,56,71,90,112,140,180,224,280,355,450,560,710,900,1120,\
                                                   1400,1800,2240,2800,3550,4500,5600,7100,9000,11200 ])
    sts.harmonics                      = np.arange(1,30)


    sts.broadband_spectrum_resolution        = 301
    sts.floating_point_precision             = np.float32
    sts.urban_canyon_microphone_z_resolution = 16 
    sts.mic_x_position                       = 0     
    sts.number_of_multiprocessing_workers    = 8
    sts.parallel_computing                   = True # TO BE REMOVED
    sts.lateral_ground_distance              = 1000 * Units.feet  
    sts.level_ground_microphone_min_x        = -50
    sts.level_ground_microphone_max_x        = 1000
    sts.level_ground_microphone_min_y        = -1000 * Units.feet 
    sts.level_ground_microphone_max_y        = 1000 * Units.feet 
    sts.level_ground_microphone_x_resolution = 16 
    sts.level_ground_microphone_y_resolution = 4      
    return sts  

 