# Imports
import SUAVE
from SUAVE.Core import Units, Data  
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_properties  import compute_airfoil_properties
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry \
     import import_airfoil_geometry     
import numpy as np  
import os 

# design propeller 

def design_BO_105_prop():          
    prop                            = SUAVE.Components.Energy.Converters.Rotor()
    prop.inputs                     = Data() 
    prop.inputs.pitch_command       = 0 
    prop.inputs.y_axis_rotation     = 0.
    prop.tag                        = 'BO_105_40_percent_scale'
    prop.tip_radius                 = 2
    prop.hub_radius                 = prop.tip_radius*0.1 
    prop.number_of_blades           = 4  
    prop.thrust_angle               = 0.
    prop.airfoil_flag               = True
    num_sec                         = 20
    delta_beta                      = 1.5 * Units.degrees 
    non_dim_r                       = np.linspace(prop.hub_radius,0.99,num_sec)
    prop.radius_distribution        = non_dim_r*prop.tip_radius
    prop.thickness_to_chord         = np.ones(num_sec)*0.12
    prop.chord_distribution         = np.ones(num_sec)*0.121 
    prop.max_thickness_distribution = prop.thickness_to_chord* prop.chord_distribution
    prop.twist_distribution         = 8*np.flip(np.linspace(0,1,num_sec))*Units.degrees + delta_beta 
    airfoil_data                    = prop.airfoil_data 
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    airfoil_data.geometry_files     =  [ rel_path +'../Airfoils/NACA_23012.txt']
    airfoil_data.polars_files       =  [[rel_path +'../Airfoils/Polars/NACA_23012_polar_Re_50000.txt',
                                         rel_path +'../Airfoils/Polars/NACA_23012_polar_Re_100000.txt',
                                         rel_path +'../Airfoils/Polars/NACA_23012_polar_Re_200000.txt',
                                         rel_path +'../Airfoils/Polars/NACA_23012_polar_Re_500000.txt',
                                         rel_path +'../Airfoils/Polars/NACA_23012_polar_Re_1000000.txt']]
    airfoil_data.polar_stations     = list(np.zeros(num_sec).astype(int))
    airfoil_data.geometry           = import_airfoil_geometry(airfoil_data.geometry_files) 
    airfoil_data.airfoil_flag       = True  
    airfoil_data.polars             = compute_airfoil_properties(airfoil_data.geometry, airfoil_data.polars_files)  
    prop.mid_chord_alignment         = np.zeros_like(prop.chord_distribution)       
    
    return prop