import SUAVE 
from SUAVE.Core import Units , Data  

# Package Imports 
import matplotlib.cm as cm 
import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker 
import matplotlib.colors as colors  
from matplotlib.cm import ScalarMappable
from mpl_toolkits.mplot3d import Axes3D  
from SUAVE.Methods.Propulsion                                          import lift_rotor_design ,prop_rotor_design , propeller_design 
from SUAVE.Analyses.Mission.Segments.Segment                           import Segment 
from SUAVE.Methods.Noise.Fidelity_One.Propeller.propeller_mid_fidelity import propeller_mid_fidelity
from SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics           import Aerodynamics 
from SUAVE.Components.Energy.Networks.Battery_Propeller                import Battery_Propeller  
from SUAVE.Components.Energy.Converters                                import Lift_Rotor, Rotor, Prop_Rotor, Propeller
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.generate_interpolated_airfoils import generate_interpolated_airfoils 

from SUAVE.Components.Energy.Networks.Battery_Propeller                import Battery_Propeller 
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry import import_airfoil_geometry
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_naca_4series    import compute_naca_4series
import os
import pickle


import sys  
sys.path.append('../S5A_Method_Convergence_Comparison') 
sys.path.append('../S5C_and_D_Rotor_Plotting_Functions') 

from lift_rotor_traditional_discretization_optimization import lift_rotor_traditional_discretization_optimization
from Results_Plots import * 

# Package Imports  
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
    rotor.hub_radius                 = 0.15 * rotor.tip_radius  
    rotor.number_of_blades           = 3    
    
    # hover 
    rotor.hover.design_thrust               = 23544/(8)  # based on Stopped-Rotor V2, vehicle weight/(number of rotors - 1 )
    rotor.hover.design_freestream_velocity  = np.sqrt(rotor.hover.design_thrust/(2*1.2*np.pi*(rotor.tip_radius**2))) # Ideal power  
    rotor.hover.design_altitude             = 20 * Units.feet   
    
    # OEI 
    rotor.OEI.design_thrust               = 23544/(6)  # based on Stopped-Rotor V2, vehicle weight/(number of rotors - 1 )
    rotor.OEI.design_freestream_velocity  = np.sqrt(rotor.OEI.design_thrust/(2*1.2*np.pi*(rotor.tip_radius**2))) # Ideal power  
    rotor.OEI.design_altitude             = 20 * Units.feet   
       
    
    airfoil                          = SUAVE.Components.Airfoils.Airfoil()    
    airfoil.coordinate_file          =  '../Airfoils/NACA_4412.txt'
    airfoil.polar_files              = ['../Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                         '../Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                         '../Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                         '../Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                         '../Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']
    rotor.append_airfoil(airfoil)   
    rotor.airfoil_polar_stations          = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]        
    
    ## ----------------------
    ##  SIMULATIONS 
    ## ----------------------
    #'''lift rotor design using Adkins and Liebeck Method''' 
    #save_figures                       = True
    #plot_rotor_geomery_and_performance = True
    #design_lift_rotor_Adkins_Leibeck_formulation(rotor,plot_rotor_geomery_and_performance,save_figures) 
    
    
    '''lift rotor design using new method '''
    alpha_weights                      = np.linspace(0,0.2,11)  
    design_lift_rotor_new_parameterization(rotor,alpha_weights) 
    
    #'''lift rotor design using baseline '''
    #alpha_weights                     = np.linspace(0,1,5) 
    #save_figure                       = True  
    #design_lift_rotor_traditional_discretization(rotor,alpha_weights,save_figure) 

    ## ----------------------
    ##  PLOTS 
    ## ----------------------  
    save_figures                       = False       
    folder_name                        = 'S5C_Lift_Rotor_Design'
    plot_rotor_blade_comparisons(rotor,folder_name,alpha_weights =alpha_weights,beta_weights = None,gamma_weights = None,add_plot_legends = False , save_figures = save_figures)    
           
    return 

# ------------------------------------------------------------------ 
# Lift Rotor Singe Point Design Point Analysis
# ------------------------------------------------------------------ 
def design_lift_rotor_new_parameterization(rotor,alpha_weights): 

    rotor.hover.design_power  = None            
    for i in range(len(alpha_weights)):  
    
        rotor.optimization_parameters.multiobjective_aeroacoustic_weight = alpha_weights[i]            
        rotor  = lift_rotor_design(rotor,print_iterations = True)  
        # save rotor geomtry
        opt_weight = str(format(alpha_weights[i],'.5f'))
        opt_weight = opt_weight.replace('.','_')    
        name       = 'LR_Alpha_' + opt_weight 
        save_blade_geometry(rotor,name)
         
        # reset power to none 
        rotor.hover.design_power  = None           
                
    return 

# ------------------------------------------------------------------ 
# Lift Rotor Singe Point Design Point Analysis
# ------------------------------------------------------------------ 
def design_lift_rotor_traditional_discretization(rotor,alpha_weights,save_figure): 
    
    for i in range(len(alpha_weights)):  
        opt_params                       = rotor.optimization_parameters 
        opt_params.aeroacoustic_weight   = alpha_weights[i]    
        
        # design rotor 
        rotor                            = lift_rotor_traditional_discretization_optimization(rotor)   
      
        # save rotor geomtry
        opt_weight = str(format(rotor.optimization_parameters.aeroacoustic_weight,'.5f'))
        opt_weight = opt_weight.replace('.','_')    
        name       = 'LR_' + str(int(rotor.design_thrust))  + '_Alpha_' + opt_weight + '_TD'
        save_blade_geometry(rotor,name)  
         
        plot_3d_rotor_geometry(rotor,name,save_figure)
         
    return 


# ------------------------------------------------------------------ 
# Stopped-Rotors Adkins and Liebeck
# ------------------------------------------------------------------ 
def design_lift_rotor_Adkins_Leibeck_formulation(rotor,plot_rotor_geomery_and_performance,save_figures):  
    
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
 
    rotor.design_SPL_dBA           = mean_SPL
    rotor.hover.design_performance = rotor_aero_data
    rotor.hover.design_moise       = rotor_noise_data 
    rotor.airfoil_flag             = True  
    rotor_tag                      ='LR_AL' 
     
    save_blade_geometry(rotor,rotor_tag)   
         
    return     

# ------------------------------------------------------------------ 
# Load data  
# ------------------------------------------------------------------     
def load_blade_geometry(filename):  
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator     
    load_file = rel_path + 'Rotor_Designs/' + filename + '.pkl'
    with open(load_file, 'rb') as file:
        rotor = pickle.load(file) 
    return rotor


# ------------------------------------------------------------------
#   Save Blade Geometry
# ------------------------------------------------------------------   
def save_blade_geometry(rotor,filename): 
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator     
    pickle_file  = rel_path + 'Rotor_Designs/' + filename + '.pkl'
    with open(pickle_file, 'wb') as file:
        pickle.dump(rotor, file) 
    return     


if __name__ == '__main__': 
    main() 
    plt.show()