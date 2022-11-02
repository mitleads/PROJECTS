import SUAVE 
from SUAVE.Core import Units , Data 
from SUAVE.Methods.Propulsion                                          import lift_rotor_design,  propeller_design 
from SUAVE.Analyses.Mission.Segments.Segment                           import Segment 
from SUAVE.Methods.Noise.Fidelity_One.Propeller.propeller_mid_fidelity import propeller_mid_fidelity
from SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics           import Aerodynamics 
from SUAVE.Components.Energy.Networks.Battery_Propeller                import Battery_Propeller 
from SUAVE.Components.Energy.Converters                                import Lift_Rotor 
from Rotor_Plotting_Functions.Rotor_Plots                              import * 

# Package Imports 
import matplotlib.cm as cm 
import numpy as np 
import matplotlib.pyplot as plt
import os
import pickle


# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main():  

    rotor                            = Lift_Rotor() 
    rotor.tag                        = 'rotor'
    rotor.orientation_euler_angles   = [0, 90*Units.degrees,0]
    rotor.tip_radius                 = 2.7/2
    rotor.hub_radius                 = 0.2 * rotor.tip_radius  
    rotor.number_of_blades           = 3    
    rotor.design_tip_mach            = 0.5 
    rotor.angular_velocity           = rotor.design_tip_mach* 343 /rotor.tip_radius 
    rotor.design_thrust              = 23544/(8-1)  # vased on Stopped-Rotor V2, vehicle weight/(number of rotors - 1 )
    rotor.freestream_velocity        = np.sqrt(rotor.design_thrust/(2*1.2*np.pi*(rotor.tip_radius**2))) # Ideal power  
    rotor.design_Cl                  = 0.7
    rotor.design_altitude            = 20 * Units.feet 
    rotor.design_microphone_angle    = 175 * Units.degrees
    rotor.design_velocity_vector     = np.array([52,0,-0.9]) 
    rotor.design_disc_plane          = 95 * Units.degrees
    rotor.variable_pitch             = True   
    airfoil                          = SUAVE.Components.Airfoils.Airfoil()    
    airfoil.coordinate_file          =  '../../Aircraft_Models/Airfoils/NACA_4412.txt'
    airfoil.polar_files              = ['../../Aircraft_Models/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                         '../../Aircraft_Models/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                         '../../Aircraft_Models/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                         '../../Aircraft_Models/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                         '../../Aircraft_Models/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']
    rotor.append_airfoil(airfoil)   
    rotor.airfoil_polar_stations          = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]        
    
 
    #'''lift rotor design using Adkins and Liebeck Method''' 
    #save_figures                       = False   
    #plot_rotor_geomery_and_performance = True
    #lift_rotor_Adkins_Leibeck(rotor,plot_rotor_geomery_and_performance,save_figures) 
    
    
    '''lift rotor design using new method '''
    alpha_weights                      = np.array([0.5])  #  np.linspace(0.0,1.0,200)    
    plot_rotor_geomery_and_performance = True
    use_pyoptsparse                    = False
    save_figures                       = False   
    design_lift_rotors(rotor,alpha_weights,use_pyoptsparse,plot_rotor_geomery_and_performance,save_figures)
    
    
    
    #'''plot lift rotor pareto fronteir '''
    #alpha_weights                      = np.array([1.0,0.75,0.5,0.25,0.0]) #  np.linspace(0.0,0.2,21)    
    #use_pyoptsparse                    = False
    #save_figures                       = False   
    #plot_lift_rotor_pareto_fronteir(rotor.design_thrust,alpha_weights,use_pyoptsparse,save_figures) 
    
    return 

# ------------------------------------------------------------------ 
# Lift Rotor Singe Point Design Point Analysis
# ------------------------------------------------------------------ 
def design_lift_rotors(rotor,alpha_weights,use_pyoptsparse_flag, plot_rotor_geomery_and_performance,save_figures): 
     
    if use_pyoptsparse_flag:
        optimizer = 'SNOPT'
    else: 
        optimizer = 'SLSQP'
    
    for i in range(len(alpha_weights)):  
        opt_params                       = rotor.optimization_parameters 
        opt_params.aeroacoustic_weight   = alpha_weights[i]    
        
        # design rotor 
        rotor                            = lift_rotor_design(rotor,solver_name=optimizer)   
      
        # save rotor geomtry
        opt_weight = str(rotor.optimization_parameters.aeroacoustic_weight)
        opt_weight = opt_weight.replace('.','_')    
        name       = 'Rotor_T_' + str(int(rotor.design_thrust))  + '_Alpha_' + opt_weight + '_Opt_' + optimizer
        save_blade_geometry(rotor,name)
        
        if  plot_rotor_geomery_and_performance: 
            plot_geoemtry_and_performance(rotor,name,save_figures)  
            
        # reset power to none 
        rotor.design_power = None 
    return 


# ------------------------------------------------------------------ 
# Stopped-Rotors Adkins and Liebeck
# ------------------------------------------------------------------ 
def lift_rotor_Adkins_Leibeck(rotor,plot_rotor_geomery_and_performance,save_figure): 
    
    # DESIGN ROTOR 
    rotor                        = propeller_design(rotor)   
    net                          = Battery_Propeller()
    net.number_of_rotor_engines  = 1                              
    net.identical_propellers     = True  
    net.propellers.append(rotor)  
       

    # Calculate atmospheric properties
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(rotor.design_altitude)   
    T              = atmo_data.temperature[0]
    rho            = atmo_data.density[0]
    a              = atmo_data.speed_of_sound[0]
    mu             = atmo_data.dynamic_viscosity[0]  
    ctrl_pts       = 1 

    # Run Conditions     
    theta  = np.array([135])*Units.degrees + 1E-1
    S      = np.maximum(rotor.design_altitude, 20*Units.feet) 
    
    # microphone locations
    positions  = np.zeros(( len(theta),3))
    for i in range(len(theta)):
        positions [i][:] = [0.0 , S*np.sin(theta[i])  ,S*np.cos(theta[i])]   
        
    # Set up for Propeller Model
    rotor.inputs.omega                                     = np.atleast_2d(rotor.angular_velocity).T
    conditions                                             = Aerodynamics()   
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * rho
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * mu
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T  
    conditions.frames.inertial.velocity_vector             = np.array([[0, 0. ,rotor.freestream_velocity]]) 
    conditions.propulsion.throttle                         = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial           = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., -1.]]])   
    # Run Propeller model 
    thrust , torque, power, Cp  , rotor_aero_data , etap   = rotor.spin(conditions)

    # Prepare Inputs for Noise Model  
    conditions.noise.total_microphone_locations            = np.repeat(positions[ np.newaxis,:,: ],1,axis=0)
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    segment.state.conditions.expand_rows(ctrl_pts) 

    # Store Noise Data 
    noise                                   = SUAVE.Analyses.Noise.Fidelity_One() 
    settings                                = noise.settings   
    num_mic                                 = len(conditions.noise.total_microphone_locations[0])  
    conditions.noise.number_of_microphones  = num_mic   

    rotor_noise_data   =  propeller_mid_fidelity(net.propellers,rotor_aero_data,segment,settings)   
    mean_SPL           =  np.mean(rotor_noise_data.SPL_dBA)  
 
    rotor.design_SPL_dBA     = mean_SPL
    rotor.design_performance = rotor_aero_data
    rotor.design_acoustics   = rotor_noise_data 
    rotor.airfoil_flag       = True  
    rotor_tag                ='Rotor_T_' + str(int(rotor.design_thrust)) + '_AL' 
    
    if save_figure:
        save_blade_geometry(rotor,rotor_tag) 
    

    if  plot_rotor_geomery_and_performance: 
        plot_geoemtry_and_performance(rotor,rotor_tag,save_figure) 
        plot_3d_rotor_geometry(rotor,rotor_tag,save_figure)   
    return    
  
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