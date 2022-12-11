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
from SUAVE.Methods.Propulsion                                          import lift_rotor_design , prop_rotor_design , propeller_design 
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
sys.path.append('../S5C_and_D_Rotor_Plotting_Functions')
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
    
    # DEFINE ROTOR OPERATING CONDITIONS 

    prop_rotor                                    = SUAVE.Components.Energy.Converters.Prop_Rotor() 
    prop_rotor.tag                                = 'prop_rotor'     
    prop_rotor.tip_radius                         = 3/2
    prop_rotor.hub_radius                         = 0.15 * prop_rotor.tip_radius
    prop_rotor.number_of_blades                   = 3  
    
    # HOVER 
    prop_rotor.hover.design_altitude              = 20 * Units.feet                  
    prop_rotor.hover.design_thrust                = 23175.5364/6 # weight of joby-like aircrft
    prop_rotor.hover.design_freestream_velocity   = np.sqrt(prop_rotor.hover.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))  
  
    # OEI 
    prop_rotor.OEI.design_thrust                  = 23175.5364/4  # based on Stopped-Rotor V2, vehicle weight/(number of rotors - 1 )
    prop_rotor.OEI.design_freestream_velocity     = np.sqrt(prop_rotor.OEI.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2))) # Ideal power  
    prop_rotor.OEI.design_altitude                = 20 * Units.feet   
    
    # CRUISE                   
    prop_rotor.cruise.design_altitude             = 2500 * Units.feet                      
    prop_rotor.cruise.design_thrust               = 4000/6
    prop_rotor.cruise.design_freestream_velocity  = 175*Units.mph 

    airfoil                                       = SUAVE.Components.Airfoils.Airfoil()    
    airfoil.coordinate_file                       =  '../Airfoils/NACA_4412.txt'
    airfoil.polar_files                           = ['../Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                                      '../Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                                      '../Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                                      '../Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                                      '../Airfoils/Polars/NACA_4412_polar_Re_1000000.txt'] 
    prop_rotor.append_airfoil(airfoil)   
    prop_rotor.airfoil_polar_stations             = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]   
     
    '''prop rotor design using new method '''
    alpha_weights                      = np.linspace(0.0,1.,21) 
    beta_weights                       = np.array([0.0])  # 0.0, 0.25, 0.5 ,0.75 , 1.0
    include_OEI_constraint             = True 
    design_prop_rotor_new_parameterization(prop_rotor,alpha_weights,beta_weights,include_OEI_constraint)  
  
    ### ----------------------
    ###  PLOTS 
    ### ---------------------- 
    #alpha_weights                      = np.array([1.0,1.0])
    #beta_weights                       = np.array([0.1,0.9])   
    #save_figures                       = False       
    #folder_name                        = 'S5D_Prop_Rotor_Design'
    #plot_rotor_blade_comparisons(prop_rotor,folder_name,alpha_weights =alpha_weights,beta_weights = beta_weights,gamma_weights = None,add_plot_legends = False , save_figures = save_figures)    
     
    return 
 

# ------------------------------------------------------------------ 
# Tiltwing Prop-Rotor Design Point Analysis
# ------------------------------------------------------------------ 
def design_prop_rotor_new_parameterization(prop_rotor,alpha_weights,beta_weights,include_OEI_constraint):  
    
    for i in range(len(alpha_weights)):
        for j in range(len(beta_weights)):

            # reset power to none 
            prop_rotor.cruise.design_power = None     
            prop_rotor.hover.design_power  = None  
            
             # OPTIMIZATION PARAMETERS  
            opt_params                                    = prop_rotor.optimization_parameters 
            opt_params.multiobjective_aeroacoustic_weight = alpha_weights[i]   # 1 means only perfomrance optimization 0.5 to weight noise equally
            opt_params.multiobjective_performance_weight  = beta_weights[j]
            opt_params.multiobjective_acoustic_weight     = 1 # Do not consider cruise noise 
                  
            # DESING ROTOR       
            prop_rotor                                    = prop_rotor_design(prop_rotor,include_OEI_constraint,print_iterations = True)  
            
            # save rotor geomtry
            alpha_opt_weight = str(format(alpha_weights[i],'.5f'))
            alpha_opt_weight = alpha_opt_weight.replace('.','_')    
            beta_opt_weight  = str(format(beta_weights[j],'.5f'))
            beta_opt_weight  = beta_opt_weight.replace('.','_')  
            if include_OEI_constraint:
                OEI_tag = '_OEI'
            else:
                OEI_tag = '_no_OEI'            
            name       = 'PR_Alpha_' + alpha_opt_weight + '_Beta_' + beta_opt_weight + OEI_tag  
            save_blade_geometry(prop_rotor,name) 
        
            # reset power to none 
            prop_rotor.cruise.design_power = None     
            prop_rotor.hover.design_power  = None                   
    
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