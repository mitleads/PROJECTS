# Stopped_Rotor_Plots.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

#----------------------------------------------------------------------
#   Imports
# --------------------------------------------------------------------- 
from SUAVE.Core import Units, Data    
from SUAVE.Plots.Performance.Mission_Plots import *  
from SUAVE.Plots.Geometry   import *  
import numpy as np
import pylab as plt  

# ----------------------------------------------------------------------
#   Plot Results
# ----------------------------------------------------------------------
def plot_results(results,run_noise_model,line_style='bo-'):
    
    # Universal Plot Settings 
    plt.rcParams['axes.linewidth'] = 2.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 24,
                  'xtick.labelsize': 20,
                  'ytick.labelsize': 20,
                  'axes.titlesize': 24}
    plt.rcParams.update(parameters)
    plot_parameters                  = Data()
    plot_parameters.line_width       = 3 
    plot_parameters.line_style       = '-' 
    plot_parameters.figure_width     = 12 
    plot_parameters.figure_height    = 6 
    plot_parameters.marker_size      = 10 
    plot_parameters.legend_font_size = 20 
    plot_parameters.plot_grid        = True   
    plot_parameters.markers          = ['o','v','s','P','p','^','D','X','*']
    plot_parameters.colors           = cm.viridis(np.linspace(0,1,5))     
    plot_parameters.lw               = 3                              # line_width               
    plot_parameters.m                = 14                             # markersize               
    plot_parameters.legend_font      = 20                             # legend_font_size         
    plot_parameters.Slc              = ['black','dimgray','silver' ]  # SUAVE_line_colors        
    plot_parameters.Slm              = '^'                            # SUAVE_line_markers       
    plot_parameters.Sls              = '-'                            # SUAVE_line_styles        
    plot_parameters.Elc              = ['firebrick','red','tomato']   # Experimental_line_colors 
    plot_parameters.Elm              = 's'                            # Experimental_line_markers
    plot_parameters.Els              = '-'                            # Experimental_line_styles 
    plot_parameters.Rlc              = ['mediumblue','blue','cyan']   # Ref_Code_line_colors     
    plot_parameters.Rlm              = 'o'                            # Ref_Code_line_markers    
    plot_parameters.Rls              = '--'                           # Ref_Code_line_styles     
     

    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style) 

    # Plot Aerodynamic  Forces
    plot_aerodynamic_forces(results)
    
    # Plot Aerodynamic Coefficents
    plot_aerodynamic_coefficients(results)

    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results, line_style)

    # Plot Aircraft Electronics
    plot_battery_pack_conditions(results, line_style)

    # Plot Propeller Conditions 
    plot_propeller_conditions(results, line_style) 

    # Plot Electric Motor and Propeller Efficiencies 
    plot_eMotor_Prop_efficiencies(results, line_style)

    # Plot propeller Disc and Power Loading
    plot_disc_power_loading(results, line_style)   

    # Plot Battery Degradation  
    plot_battery_degradation(results, line_style)       
    
    if run_noise_model:   
        # Plot noise level
        plot_ground_noise_levels(results)
        
        # Plot noise contour
        plot_flight_profile_noise_contours(results)  

    return 

