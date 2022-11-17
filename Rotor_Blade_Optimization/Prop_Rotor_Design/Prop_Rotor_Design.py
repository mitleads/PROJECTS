import SUAVE 
from SUAVE.Core import Units , Data 
from SUAVE.Methods.Propulsion                                          import lift_rotor_design , prop_rotor_design , propeller_design 
from SUAVE.Analyses.Mission.Segments.Segment                           import Segment 
from SUAVE.Methods.Noise.Fidelity_One.Propeller.propeller_mid_fidelity import propeller_mid_fidelity
from SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics           import Aerodynamics 
from SUAVE.Components.Energy.Networks.Battery_Propeller                import Battery_Propeller 
from SUAVE.Plots.Geometry.plot_vehicle                                 import plot_propeller_geometry 
from SUAVE.Components.Energy.Converters                                import Lift_Rotor, Rotor, Prop_Rotor, Propeller

import sys 
sys.path.append('../Rotor_Plotting_Functions')
from Rotor_Plotting_Functions.Rotor_Plots import * 

# Package Imports  
import numpy as np 
import matplotlib.pyplot as plt 
import os
import pickle

# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main(): 
    
    # DEFINE ROTOR OPERATING CONDITIONS 

    prop_rotor                                    = SUAVE.Components.Energy.Converters.Prop_Rotor() 
    prop_rotor.tag                                = 'prop_rotor'     
    prop_rotor.tip_radius                         = 3/2
    prop_rotor.hub_radius                         = 0.15 * prop_rotor.tip_radius
    prop_rotor.number_of_blades                   = 3  
    
    # HOVER 
    prop_rotor.design_altitude_hover              = 20 * Units.feet                  
    prop_rotor.design_thrust_hover                = (2362.44*9.81)/(6) # weight of joby-like aircrft
    prop_rotor.freestream_velocity_hover          = np.sqrt(prop_rotor.design_thrust_hover/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))  
 
    # CRUISE                   
    prop_rotor.design_altitude_cruise             = 2500 * Units.feet                      
    prop_rotor.design_thrust_cruise               = 4000/6
    prop_rotor.freestream_velocity_cruise         = 175*Units.mph 

    airfoil                                       = SUAVE.Components.Airfoils.Airfoil()    
    airfoil.coordinate_file                       =  '../../Aircraft_Models/Airfoils/NACA_4412.txt'
    airfoil.polar_files                           = ['../../Aircraft_Models/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                                      '../../Aircraft_Models/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                                      '../../Aircraft_Models/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                                      '../../Aircraft_Models/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                                      '../../Aircraft_Models/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt'] 
    prop_rotor.append_airfoil(airfoil)   
    prop_rotor.airfoil_polar_stations             = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]   
    
    #'''prop rotor design using Adkins and Liebeck Method'''  
    #plot_rotor_geomery_and_performance = True 
    #save_figures                       = False  
    #prop_rotor_Adkins_Leibeck(prop_rotor,plot_rotor_geomery_and_performance,save_figures) 
    
    '''prop rotor design using new method '''
    alpha_weights                      = np.linspace(0.0,1.,21) 
    beta_weights                       = np.linspace(0.0,0.25,6) #[0,0.25,0.5,0.75,1]
    use_pyoptsparse                    = False 
    plot_rotor_geomery_and_performance = False 
    save_figures                       = False 
    design_prop_rotors(prop_rotor,alpha_weights,beta_weights,use_pyoptsparse, plot_rotor_geomery_and_performance,save_figures)  
   
    #'''plot prop rotor pareto fronteir '''  
    #plot_rotor_geomery_and_performance = False
    #use_pyoptsparse                    = False
    #save_figures                       = False   
    #prop_rotor_designs_and_pareto_fronteir(alpha_weights,np.ones_like(alpha_weights)*0.5,'Alpha_Sweep',use_pyoptsparse,save_figures)
    #prop_rotor_designs_and_pareto_fronteir(np.ones_like(beta_weights)*0.625,beta_weights,'Beta_Sweep',use_pyoptsparse,save_figures)
     
    #'''compare two rotors designs'''
    #alpha_weights                      = np.array([1.0,1.0])
    #beta_weights                       = np.array([0.1,0.9])   
    #use_pyoptsparse                    = False
    #save_figures                       = False    
    #plot_prop_rotor_design_comparisons(alpha_weights,beta_weights,use_pyoptsparse,save_figures) 
        
    
    return 
 

# ------------------------------------------------------------------ 
# Tiltwing Prop-Rotor Design Point Analysis
# ------------------------------------------------------------------ 
def design_prop_rotors(prop_rotor,alpha_weights,beta_weights,use_pyoptsparse_flag, plot_rotor_geomery_and_performance,save_figures): 
     
    if use_pyoptsparse_flag:
        optimizer = 'SNOPT'
    else: 
        optimizer = 'SLSQP'
    
    for i in range(len(alpha_weights)):
        for j in range(len(beta_weights)):
            
             # OPTIMIZATION PARAMETERS  
            opt_params                                    = prop_rotor.optimization_parameters 
            opt_params.multiobjective_performance_weight  = beta_weights[j]
            opt_params.multiobjective_acoustic_weight     = 1 # Do not consider cruise noise 
            opt_params.aeroacoustic_weight                = alpha_weights[i]   # 1 means only perfomrance optimization 0.5 to weight noise equally
                  
            # DESING ROTOR       
            prop_rotor                                    = prop_rotor_design(prop_rotor,solver_name=optimizer)   
          
            # save rotor geomtry
            alpha_opt_weight = str(alpha_weights[i])
            alpha_opt_weight = alpha_opt_weight.replace('.','_')    
            beta_opt_weight  = str(beta_weights[j])
            beta_opt_weight  = beta_opt_weight.replace('.','_')    
            name       = 'Rotor_TH_' + str(int(prop_rotor.design_thrust_hover)) + '_TC_' + str(int(prop_rotor.design_thrust_cruise)) +\
                          '_Alpha_' + alpha_opt_weight + '_Beta_' + beta_opt_weight + '_Opt_' + optimizer
            save_blade_geometry(prop_rotor,name)
            
            if plot_rotor_geomery_and_performance: 
                plot_geoemtry_and_performance(prop_rotor,name,save_figures) 
        
            # reset power to none 
            prop_rotor.design_power = None                 
    
    return  



# ------------------------------------------------------------------ 
# Tiltwing Prop-Rotor Adkins and Liebeck
# ------------------------------------------------------------------ 
def prop_rotor_Adkins_Leibeck(prop_rotor,plot_rotor_geomery_and_performance,save_figure):  
 
    prop_rotor.design_tip_mach              = 0.6 
    prop_rotor.angular_velocity             = prop_rotor.design_tip_mach*343/prop_rotor.tip_radius  
    prop_rotor.freestream_velocity          = prop_rotor.freestream_velocity_hover
    prop_rotor.design_Cl                    = 0.7
    prop_rotor.design_altitude              = prop_rotor.design_altitude_hover          
    prop_rotor.design_thrust                = prop_rotor.design_thrust_hover 
    
    prop_rotor                              = propeller_design(prop_rotor)   
 
    net                                     = Battery_Propeller()
    net.number_of_propeller_engines         = 1                              
    net.identical_propellers                = True  
    net.propellers.append(prop_rotor)   
    # ------------------------------------------------------------------
    # HOVER PERFORMANCE
    # ------------------------------------------------------------------
    omega                           = prop_rotor.angular_velocity_hover
    prop_rotor.inputs.pitch_command =  0.  * Units.degrees
    V                               = prop_rotor.freestream_velocity_hover  
    alt                             = prop_rotor.design_altitude_hover   
    
    # Calculate atmospheric properties
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(alt)   
    T              = atmo_data.temperature[0]
    rho            = atmo_data.density[0]
    a              = atmo_data.speed_of_sound[0]
    mu             = atmo_data.dynamic_viscosity[0]  
    ctrl_pts       = 1 
    
    # Run Conditions       
    theta  = np.array([135])*Units.degrees 
    S      = np.maximum(alt , 20*Units.feet) 
    
    # microphone locations
    positions  = np.zeros(( len(theta),3))
    for i in range(len(theta)):
        positions [i][:] = [0.0 , S*np.sin(theta[i])  ,S*np.cos(theta[i])]   
            
    # Set up for Propeller Model
    prop_rotor.inputs.omega                          = np.atleast_2d(omega).T
    conditions                                       = Aerodynamics()   
    conditions.freestream.density                    = np.ones((ctrl_pts,1)) * rho
    conditions.freestream.dynamic_viscosity          = np.ones((ctrl_pts,1)) * mu
    conditions.freestream.speed_of_sound             = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                = np.ones((ctrl_pts,1)) * T 
    conditions.frames.inertial.velocity_vector       = np.array([[0, 0. ,V]]) 
    conditions.propulsion.throttle                   = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial     = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., -1.]]]) 
    
    # Run Propeller model 
    thrust , torque, power, Cp_hover  , rotor_aero_data_hover , etap  = prop_rotor.spin(conditions)
    
    # Prepare Inputs for Noise Model  
    conditions.noise.total_microphone_locations            = np.repeat(positions[ np.newaxis,:,: ],1,axis=0)
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    segment.state.conditions.expand_rows(ctrl_pts) 
    
    # Store Noise Data 
    noise                                                  = SUAVE.Analyses.Noise.Fidelity_One() 
    settings                                               = noise.settings   
    num_mic                                                = len(conditions.noise.total_microphone_locations[0])  
    conditions.noise.number_of_microphones                 = num_mic   
    
    rotor_noise_hover   = propeller_mid_fidelity(net.propellers,rotor_aero_data_hover,segment,settings)   
    mean_SPL_hover          = np.mean(rotor_noise_hover.SPL_dBA) 
    
    if prop_rotor.design_power_hover == None: 
        prop_rotor.design_power_hover = power[0][0]
    if prop_rotor.design_thrust_hover == None: 
        prop_rotor.design_thrust_hover = thrust[0][0]
        
    design_torque_hover = power[0][0]/omega
    
    
    # ------------------------------------------------------------------
    # CRUISE PERFORMANCE
    # ------------------------------------------------------------------
    omega                           = prop_rotor.angular_velocity_cruise 
    prop_rotor.inputs.pitch_command = 0.  * Units.degrees 
    V                               = prop_rotor.freestream_velocity_cruise  
    alt                             = prop_rotor.design_altitude_cruise    
    
    # Calculate atmospheric properties
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(alt)   
    T              = atmo_data.temperature[0]
    rho            = atmo_data.density[0]
    a              = atmo_data.speed_of_sound[0]
    mu             = atmo_data.dynamic_viscosity[0]  
    ctrl_pts       = 1 
    
    # Run Conditions     
    theta     = np.array([135])*Units.degrees + 1E-4
    S         = np.maximum(alt , 20*Units.feet)
    ctrl_pts  = 1 
    positions = np.zeros(( len(theta),3))
    for i in range(len(theta)):
        positions[i][:] = [S*np.cos(theta[i]) ,S*np.sin(theta[i]),-alt]
    
    # Set up for Propeller Model
    prop_rotor.inputs.omega                                = np.atleast_2d(omega).T
    conditions                                             = Aerodynamics()   
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * rho
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * mu
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T 
    conditions.frames.inertial.velocity_vector             = np.array([[V, 0. ,0.]])
    conditions.propulsion.throttle                         = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial           = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])
    
    # Run Propeller model 
    thrust , torque, power, Cp_cruise  , rotor_aero_data_cruise , etap  = prop_rotor.spin(conditions)
    
    # Prepare Inputs for Noise Model  
    conditions.noise.total_microphone_locations            = np.repeat(positions[ np.newaxis,:,: ],1,axis=0)
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    segment.state.conditions.expand_rows(ctrl_pts) 
    
    # Store Noise Data 
    noise                                                  = SUAVE.Analyses.Noise.Fidelity_One() 
    settings                                               = noise.settings   
    num_mic                                                = len(conditions.noise.total_microphone_locations[0])  
    conditions.noise.number_of_microphones                 = num_mic   
    
    rotor_noise_cruise   = propeller_mid_fidelity(net.propellers,rotor_aero_data_cruise,segment,settings)   
    mean_SPL_cruise      = np.mean(rotor_noise_cruise.SPL_dBA) 
    
    if prop_rotor.design_power_cruise == None: 
        prop_rotor.design_power_cruise = power[0][0]
    if prop_rotor.design_thrust_cruise == None: 
        prop_rotor.design_thrust_cruise = -thrust[0][2] 
        
    design_torque_cruise = power[0][0]/omega
    
    prop_rotor.design_torque_hover               = design_torque_hover
    prop_rotor.design_torque_cruise              = design_torque_cruise  
    prop_rotor.design_power_coefficient_hover    = Cp_hover[0][0] 
    prop_rotor.design_power_coefficient_cruise   = Cp_cruise[0][0] 
    prop_rotor.design_thrust_coefficient_hover   = rotor_aero_data_hover.thrust_coefficient[0][0] 
    prop_rotor.design_thrust_coefficient_cruise  = rotor_aero_data_cruise.thrust_coefficient[0][0] 
    prop_rotor.design_SPL_dBA_hover              = mean_SPL_hover
    prop_rotor.design_SPL_dBA_cruise             = mean_SPL_cruise
    prop_rotor.design_SPL_dBA                    = mean_SPL_cruise
    prop_rotor.design_performance_hover          = rotor_aero_data_hover
    prop_rotor.design_performance_cruise         = rotor_aero_data_cruise
    prop_rotor.design_acoustics_hover            = rotor_noise_hover
    prop_rotor.design_acoustics_cruise           = rotor_noise_cruise 
    prop_rotor.design_performance                = rotor_aero_data_hover        
    prop_rotor.design_acoustics                  = rotor_noise_hover 
    rotor_tag                                    = 'Rotor_T_' + str(int(prop_rotor.design_thrust)) + '_AL' 
    
    if save_figure:
        save_blade_geometry(prop_rotor,rotor_tag) 
    
    
    if  plot_rotor_geomery_and_performance: 
        plot_geoemtry_and_performance(prop_rotor,rotor_tag,save_figure) 
        plot_3d_rotor_geometry(prop_rotor,rotor_tag,save_figure)
            
    return  


 
# ------------------------------------------------------------------ 
# Setup Axes 
# ------------------------------------------------------------------ 
def set_up_axes(PP,design_thrust):
    
    # ------------------------------------------------------------------
    #   Twist Distribition
    # ------------------------------------------------------------------
    fig_1_name = "Rotor_Twist_Comparson_" + str(int(design_thrust))  + '_N'
    fig_1 = plt.figure(fig_1_name)
    fig_1.set_size_inches(PP.figure_width,PP.figure_height)
    axis_1 = fig_1.add_subplot(1,1,1)
    axis_1.set_ylabel(r'$\beta$ ($\degree$)') 
    axis_1.set_xlabel('r')    
    axis_1.minorticks_on()   
    
    # ------------------------------------------------------------------
    #   Chord Distribution
    # ------------------------------------------------------------------ 
    fig_2_name = "Rotor_Chord_Comparson_" + str(int(design_thrust))  + '_N'
    fig_2 = plt.figure(fig_2_name)     
    fig_2.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_2 = fig_2.add_subplot(1,1,1)  
    axis_2.set_ylabel('c (m)') 
    axis_1.set_xlabel('r')    
    axis_2.minorticks_on()    

    # ------------------------------------------------------------------
    #  Thickness Distribution
    # ------------------------------------------------------------------ 
    fig_3_name = "Rotor_Thickness_Comparson_" + str(int(design_thrust))  + '_N'
    fig_3 = plt.figure(fig_3_name)     
    fig_3.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_3 = fig_3.add_subplot(1,1,1)  
    axis_3.set_ylabel('t (m)') 
    axis_1.set_xlabel('r')    
    axis_3.minorticks_on()  

    # ------------------------------------------------------------------
    #  Thickness Distribution
    # ------------------------------------------------------------------ 
    fig_4_name = "Rotor_Hover_Power_Noise_Pareto_" + str(int(design_thrust))  + '_N'
    fig_4 = plt.figure(fig_4_name)     
    fig_4.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_4 = fig_4.add_subplot(1,1,1)  
    axis_4.set_xlabel('Power (kW)') 
    axis_4.set_ylabel('SPL (dBA)')    
    axis_4.minorticks_on()  
    
    # ------------------------------------------------------------------
    #  Spanwise Re Distribution
    # ------------------------------------------------------------------ 
    fig_5_name = "Rotor_Spanwise_Re_" + str(int(design_thrust))  + '_N'
    fig_5 = plt.figure(fig_5_name)     
    fig_5.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_5 = fig_5.add_subplot(1,1,1)  
    axis_5.set_ylabel(r'Sectional Re.') 
    axis_5.set_xlabel('r')    
    axis_5.minorticks_on()   
     

    # ------------------------------------------------------------------
    # Spanwise AoA
    # ------------------------------------------------------------------ 
    fig_6_name = "Rotor_Spanwise_AoA_" + str(int(design_thrust))  + '_N'
    fig_6 = plt.figure(fig_6_name)     
    fig_6.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_6 = fig_6.add_subplot(1,1,1)  
    axis_6.set_ylabel(r'Sectional AoA ($\degree$)') 
    axis_6.set_xlabel('r')      
    axis_6.minorticks_on()     
      

    # ------------------------------------------------------------------
    # Total SPL Spectrum Comparison
    # ------------------------------------------------------------------      
    fig_7 = plt.figure('Rotor_Total_SPL_Comparison')    
    fig_7.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_7 = fig_7.add_subplot(1,1,1)    
    axis_7.set_xscale('log') 
    axis_7.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axis_7.set_xlabel('Frequency (Hz)') 
    axis_7.set_ylim([0,100])


    # ------------------------------------------------------------------
    # Harmonic Noise Spectrum Comparison
    # ------------------------------------------------------------------  
    fig_8 = plt.figure('Rotor_Harmonic_Noise_Comparison') 
    fig_8.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_8 = fig_8.add_subplot(1,1,1)      
    axis_8.set_xscale('log')
    axis_8.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axis_8.set_xlabel('Frequency (Hz)')  
    axis_8.set_ylim([0,100]) 
    

    # ------------------------------------------------------------------
    # Broadband Noise Spectrum Comparison
    # ------------------------------------------------------------------      
    fig_9 = plt.figure('Rotor_Broadband_Noise_Comparison')    
    fig_9.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_9 = fig_9.add_subplot(1,1,1)    
    axis_9.set_xscale('log')
    axis_9.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axis_9.set_xlabel('Frequency (Hz)') 
    axis_9.set_ylim([0,100])
    

    # ------------------------------------------------------------------
    #  Thickness Distribution
    # ------------------------------------------------------------------ 
    fig_10_name = "Rotor_Cruise_Power_Noise_Pareto_" + str(int(design_thrust))  + '_N'
    fig_10 = plt.figure(fig_10_name)     
    fig_10.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_10 = fig_10.add_subplot(1,1,1)  
    axis_10.set_xlabel('Power (kW)') 
    axis_10.set_ylabel('SPL (dBA)')    
    axis_10.minorticks_on()   
    
    AXES    = [axis_1,axis_2,axis_3,axis_4,axis_5,axis_6,axis_7,axis_8,axis_9,axis_10]
    FIGURES = [fig_1,fig_2,fig_3,fig_4,fig_5,fig_6,fig_7,fig_8,fig_9,fig_10]
    return AXES , FIGURES

  

def define_plot_parameters(): 

    plt.rcParams['axes.linewidth'] = 2.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 32,
                  'legend.fontsize': 22,
                  'xtick.labelsize': 28,
                  'ytick.labelsize': 28,
                  'axes.titlesize': 32}
    plt.rcParams.update(parameters)
    plot_parameters                  = Data()
    plot_parameters.line_width       = 2
    plot_parameters.line_styles      = ['--',':','-',':','--']
    plot_parameters.figure_width     = 10
    plot_parameters.figure_height    = 7
    plot_parameters.marker_size      = 10
    plot_parameters.legend_font_size = 20
    plot_parameters.plot_grid        = True   

    plot_parameters.colors           = [['black','firebrick','darkblue'],
                                        ['dimgray','red','blue'], 
                                        ['darkgray','salmon','deepskyblue']]  
     
    plot_parameters.markers          = ['o','v','s','P','p','^','D','X','*']   
    
    return plot_parameters

# ------------------------------------------------------------------
#   Save Blade Geometry
# ------------------------------------------------------------------   
def save_blade_geometry(rotor,filename):
    pickle_file  = 'Rotor_Designs/' + filename + '.pkl'
    with open(pickle_file, 'wb') as file:
        pickle.dump(rotor, file) 
    return      


if __name__ == '__main__': 
    main() 
    plt.show()