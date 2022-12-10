
#----------------------------------------------------------------------
#   Imports
# --------------------------------------------------------------------- 
import SUAVE  
from SUAVE.Core          import Units, Data    

import uncertainpy       as un
import chaospy           as cp                       # To create distributions
import numpy             as np                         # For the time array 
import matplotlib.cm     as cm 
import matplotlib.pyplot as plt 
from  UQ_Plots           import plot_uq_results 
from SUAVE.Analyses.Mission.Segments.Segment                                                   import Segment 
from SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics                                   import Aerodynamics 
from SUAVE.Methods.Noise.Fidelity_One.Propeller.propeller_mid_fidelity                         import propeller_mid_fidelity
from SUAVE.Components.Energy.Networks.Battery_Propeller                                        import Battery_Propeller 
import matplotlib.pyplot as plt 
import time  
import pickle

def main():    
    
    
    ti                    = time.time() 
    
    # load optimizal lift rotor
    # Create the distributions around zero mean   
    Vinf        = cp.Normal(14.634598273606093, 14.634598273606093*0.1)  
    chord_p_val = cp.Normal(0.72094772 , 0.1)  
    chord_q_val = cp.Normal(0.96767157 , 0.1)  
    beta_p_val  = cp.Normal(0.7875114  , 0.1)  
    beta_q_val  = cp.Normal(1.5  , 0.1)               
 
    # Define the parameter dictionary 
    parameters = {"V_inf"   : Vinf, 
                  "chord_p" : chord_p_val,
                  "chord_q" : chord_q_val,
                  "beta_p"  : beta_p_val,
                  "beta_q"  : beta_q_val }
    
    UQ_Res = {}  

    #V_inf   = 14.63
    #chord_p = 0.720
    #chord_q = 0.967
    #beta_p  = 0.787
    #beta_q  = 1.5  
    #prop_model_thrust(V_inf,chord_p,chord_q,beta_p,beta_q)

    # Create a model from the battery_cell_simulation function and add labels 
    T_model   = un.Model(run=prop_model_thrust, labels=["RPM", "Thurst (N)"])    
    #SPL_model = un.Model(run=prop_model_rpm, labels=["RPM", "SPL (dBA)"])    
    #P_model   = un.Model(run=prop_model_power, labels=["RPM", "Power (kW)"])  
    #FM_model  = un.Model(run=prop_model_fm, labels=["RPM", "FM"])        
    
    # Set up the uncertainty quantification
    T_UQ   = un.UncertaintyQuantification(model= T_model, parameters=parameters)  
    #SPL_UQ = un.UncertaintyQuantification(model= SPL_model, parameters=parameters)  
    #P_UQ   = un.UncertaintyQuantification(model= P_model, parameters=parameters)  
    #FM_UQ  = un.UncertaintyQuantification(model= FM_model, parameters=parameters)  
    
    # Perform the uncertainty quantification using polynomial chaos with point collocatio  
    UQ_Res['Thrust']  = T_UQ.quantify(seed=10,polynomial_order=4,nr_pc_mc_samples=10**3)
    #UQ_Res['SPL']  = SPL_UQ.quantify(seed=10,polynomial_order=4,nr_pc_mc_samples=10**3)
    #UQ_Res['Power']  = P_UQ.quantify(seed=10,polynomial_order=4,nr_pc_mc_samples=10**3)
    #UQ_Res['FM']  = FM_UQ.quantify(seed=10,polynomial_order=4,nr_pc_mc_samples=10**3) 
    
    plot_uq_results(UQ_Res)  

    tf = time.time()
    dt = tf-ti
    print('Time Elapsed ' + str(dt/60) + ' min')        
    
    return 

# --------------------------------------------------------------
# Singe Dischage Battery Energy Tests 
# --------------------------------------------------------------
def prop_model_thrust(V_inf,chord_p,chord_q,beta_p,beta_q):  
    thrust,power,FM,perfomrance_data,noise_data = rotor_simulation(V_inf,chord_p,chord_q,beta_p,beta_q)
    omega    =  perfomrance_data.omega[:,0]            
    thrust   =  np.linalg.norm(thrust,axis =1)
    return omega,thrust 
 
def rotor_simulation(V_inf,chord_p,chord_q,beta_p,beta_q): 
    
    # load rotor  
    file_name        = 'Rotor_T_2943_Alpha_1_0_Opt_SLSQP'
    rotor            = load_blade_geometry(file_name)     
    airfoils         = rotor.Airfoils
    a_loc            = rotor.airfoil_polar_stations   
    alt              = rotor.design_altitude
    omega            = np.linspace(150,200,11) 
    ctrl_pts         = len(omega) 
    theta            = rotor.optimization_parameters.microphone_evaluation_angle = 135 * Units.degrees  # 45 degrees behind rotor plane       
    
    # Update geometry of blade
    c         = updated_blade_geometry(rotor.radius_distribution/rotor.tip_radius ,rotor.chord_r,rotor.chord_p,rotor.chord_q,rotor.chord_t)     
    beta      = updated_blade_geometry(rotor.radius_distribution/rotor.tip_radius ,rotor.twist_r,rotor.twist_p,rotor.twist_q,rotor.twist_t)   
  
    # compute max thickness distribution  
    t_max  = np.zeros(len(c))     
    if len(airfoils.keys())>0:
        for j,airfoil in enumerate(airfoils): 
            a_geo         = airfoil.geometry
            locs          = np.where(np.array(a_loc) == j )
            t_max[locs]   = a_geo.max_thickness*c[locs]   
    
    rotor.max_thickness_distribution = t_max
    rotor.chord_distribution         = c
    rotor.twist_distribution         = beta  
    rotor.mid_chord_alignment        = c/4. - c[0]/4. 
         

    # Define Network 
    net                                = Battery_Propeller()     
    net.number_of_propeller_engines    = 1
    net.identical_propellers           = True 
    net.propellers.append(rotor)    
      
    # Find the operating conditions
    # Define microphone locations 
    S              = np.maximum(alt , 20*Units.feet) 
    positions      = np.array([[0.0 , S*np.sin(theta)  ,S*np.cos(theta)]]) 

    # Calculate atmospheric properties
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(alt)     
    
    # Define run conditions 
    rotor.inputs.omega                               = np.atleast_2d(omega).T
    conditions                                       = Aerodynamics()   
    conditions.freestream.update(atmo_data) 
    conditions.frames.inertial.velocity_vector       = np.tile(np.array([[0, 0. ,V_inf]])[:,:],(ctrl_pts,1))
    conditions.propulsion.throttle                   = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial     = np.tile(np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., -1.]]])[:,:,:],(ctrl_pts,1,1))   

    # Run rotor model 
    thrust ,_, power, Cp  , perfomrance_data , _ = rotor.spin(conditions) 

    # Set up noise model
    conditions.noise.total_microphone_locations      = np.repeat(positions[ np.newaxis,:,: ],ctrl_pts,axis=0)
    conditions.aerodynamics.angle_of_attack          = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    segment                                          = Segment() 
    segment.state.conditions                         = conditions
    segment.state.conditions.expand_rows(ctrl_pts)  
    noise                                            = SUAVE.Analyses.Noise.Fidelity_One() 
    settings                                         = noise.settings   
    num_mic                                          = len(conditions.noise.total_microphone_locations[0])  
    conditions.noise.number_of_microphones           = num_mic
     
    noise_data =  propeller_mid_fidelity(net.propellers,perfomrance_data,segment,settings)     

    C_t_UIUC                = perfomrance_data.thrust_coefficient 
    C_t_rot                 = C_t_UIUC*8/(np.pi**3)
    C_p_UIUC                = Cp 
    C_q_UIUC                = C_p_UIUC/(2*np.pi) 
    C_q_rot                 = C_q_UIUC*16/(np.pi**3)   
    C_p_rot                 = C_q_rot  
    FM                      = ((C_t_rot**1.5)/np.sqrt(2))/C_p_rot 
    
    return thrust,power,FM,perfomrance_data,noise_data 


def updated_blade_geometry(chi,c_r,p,q,c_t):
    """ Computes planform function of twist and chord distributron using hyperparameters  
          
          Inputs:  
             chi - rotor radius distribution [None]
             c_r - hyperparameter no. 1      [None]
             p   - hyperparameter no. 2      [None]
             q   - hyperparameter no. 3      [None]
             c_t - hyperparameter no. 4      [None]
              
          Outputs:  
             x_lin  - function distribution  [None]
              
          Assumptions: 
             N/A 
        
          Source:
              Traub, Lance W., et al. "Effect of taper ratio at low reynolds number."
              Journal of Aircraft 52.3 (2015): 734-747.
              
    """           
    b       = chi[-1]
    r       = len(chi)                
    n       = np.linspace(r-1,0,r)          
    theta_n = n*(np.pi/2)/r              
    y_n     = b*np.cos(theta_n)          
    eta_n   = np.abs(y_n/b)            
    x_cos   = c_r*(1 - eta_n**p)**q + c_t*eta_n 
    x_lin   = np.interp(chi,eta_n, x_cos)  
    return x_lin 

# ------------------------------------------------------------------ 
# Load data  
# ------------------------------------------------------------------     
def load_blade_geometry(filename):  
    load_file = filename + '.pkl' 
    with open(load_file, 'rb') as file:
        rotor = pickle.load(file) 
    return rotor

if __name__ == '__main__':
    main()
    plt.show()
