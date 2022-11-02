# Imports
import SUAVE
from SUAVE.Core import Units, Data  
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_properties  import compute_airfoil_properties
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry \
     import import_airfoil_geometry    
from scipy.interpolate import interp1d
import os
import numpy as np  

# design propeller 
                                    
def design_Hubbard_prop(): 
    prop                            = SUAVE.Components.Energy.Converters.Propeller()
    prop.inputs                     = Data()
    prop.inputs.pitch_command       = 0 
    prop.inputs.y_axis_rotation     = 0.
    prop.tag                        = 'Hubbard_Propeller'  
    prop.tip_radius                 = 0.6096
    prop.hub_radius                 = prop.tip_radius*0.3 
    prop.number_of_blades           = 2  
    r_R                             = np.array([0.3,  0.35, 0.45,0.6 , 0.7  ,0.8 , 0.85 ,0.9  , 0.99 ])
    t_b                             = np.array([0.184931507,0.159452055,0.12109589,0.088493151,0.075068493,
                                                0.065205479,0.060821918,0.056712329,0.049589041 ])
    b_D                             = np.array([ 0.091506849,0.091232877,0.089863014,
                                                0.083835616,0.077260274,
                                                0.069315068,0.065205479,0.06109589,0.052876712])
    beta                            = np.array([ 0.089315068,0.07890411,0.064383562,
                                                 0.050958904,0.044931507,
                                                 0.040547945,0.038630137,0.036986301,0.034246575])*500 
    
    
    dim = 20                                
    new_radius_distribution         = np.linspace(0.3,0.98,dim )
    func_twist_distribution         = interp1d(r_R, (1.528 + beta )*Units.degrees , kind='cubic')
    func_chord_distribution         = interp1d(r_R,  b_D*2*prop.tip_radius       , kind='cubic')
    func_radius_distribution        = interp1d(r_R, r_R *prop.tip_radius    , kind='cubic')
    func_max_thickness_distribution = interp1d(r_R, t_b*b_D*2*prop.tip_radius  , kind='cubic')  
    
    prop.twist_distribution         = func_twist_distribution(new_radius_distribution)     
    prop.chord_distribution         = func_chord_distribution(new_radius_distribution)         
    prop.radius_distribution        = func_radius_distribution(new_radius_distribution)        
    prop.max_thickness_distribution = func_max_thickness_distribution(new_radius_distribution) 
    prop.thickness_to_chord         = prop.max_thickness_distribution/prop.chord_distribution   
    prop.mid_chord_alignment        = np.zeros_like(prop.chord_distribution)     
    airfoil_data                    = prop.airfoil_data 
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    airfoil_data.geometry_files     =  [ rel_path +'../Airfoils/Clark_y.txt']
    airfoil_data.polars_files       =  [[rel_path +'../Airfoils/Polars/Clark_y_polar_Re_50000.txt',
                                         rel_path +'../Airfoils/Polars/Clark_y_polar_Re_100000.txt',
                                         rel_path +'../Airfoils/Polars/Clark_y_polar_Re_200000.txt',
                                         rel_path +'../Airfoils/Polars/Clark_y_polar_Re_500000.txt',
                                         rel_path +'../Airfoils/Polars/Clark_y_polar_Re_1000000.txt']] 
    airfoil_data.polar_stations     = list(np.zeros(dim).astype(int) )  
    airfoil_data.geometry           = import_airfoil_geometry(airfoil_data.geometry_files) 
    airfoil_data.airfoil_flag       = True  
    airfoil_data.polars             = compute_airfoil_properties(airfoil_data.geometry, airfoil_data.polars_files)      
    
    return prop