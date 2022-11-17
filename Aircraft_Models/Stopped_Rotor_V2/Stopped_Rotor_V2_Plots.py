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

import time 
import os
import numpy as np
import pylab as plt  

# ----------------------------------------------------------------------
#   Plot Results
# ----------------------------------------------------------------------
def plot_results(results,run_noise_model,save_figure_flag,line_style='bo-'):
    
    # Universal Plot Settings 
    plt.rcParams['axes.linewidth'] = 1.
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
    plot_flight_conditions(results, line_style,save_figure = save_figure_flag)  
    
    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results, line_style,save_figure = save_figure_flag)  
    
    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results, line_style,save_figure = save_figure_flag) 

    # Plot Aircraft Electronics
    plot_battery_pack_conditions(results, line_style,save_figure = save_figure_flag)    
    
    # Plot Electric Motor and Propeller Efficiencies  of Lift Cruise Network
    plot_lift_cruise_network(results, line_style,save_figure = save_figure_flag)   

    # Plot Battery Degradation  
    plot_battery_degradation(results, line_style,save_figure = save_figure_flag)    

    if run_noise_model:     
        # Plot noise level
        plot_ground_noise_levels(results,save_figure = save_figure_flag)
        
        # Plot noise contour
        plot_flight_profile_noise_contours(results,save_figure = save_figure_flag) 
    return     
 

# ------------------------------------------------------------------
#   Set Axis Parameters 
# ------------------------------------------------------------------
## @ingroup Plots
def set_axes(axes):
    """This sets the axis parameters for all plots

    Assumptions:
    None

    Source:
    None

    Inputs
    axes
        
    Outputs: 
    axes

    Properties Used:
    N/A	
    """   
    
    axes.minorticks_on()
    axes.grid(which='major', linestyle='-', linewidth=0.5, color='grey')
    axes.grid(which='minor', linestyle=':', linewidth=0.5, color='grey')      
    axes.grid(True)   
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)        

    return  
