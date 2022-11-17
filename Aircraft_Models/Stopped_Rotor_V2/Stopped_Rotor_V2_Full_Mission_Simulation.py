# Stopped_Rotor_Full_Mission.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data   
import pickle
from SUAVE.Plots.Performance.Mission_Plots import *  
from SUAVE.Plots.Geometry   import *  

import time  
import numpy as np
import pylab as plt 

import Stopped_Rotor_V2_Vehicle
import Stopped_Rotor_V2_Analyses 
import Stopped_Rotor_V2_Missions
import Stopped_Rotor_V2_Plots 


try:
    import vsp 
    from SUAVE.Input_Output.OpenVSP.vsp_write import write 
except ImportError:
    # This allows SUAVE to build without OpenVSP
    pass  


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main(): 

    # start simulation clock
    ti                = time.time() 
    
    # SET UP SIMULATION PARAMETERS     
    simulated_days    = 1               # number of days simulated 
    flights_per_day   = 1               # number of flights per day 
    aircraft_range    = 30 *Units.nmi   # total ground distance 
    reserve_segment   = False           # reserve segment flag, if true, ground distance not included  
    recharge_battery  = False           # flag to simulate battery recharge  
    plot_mission      = True            # plot mission flag  
    control_points    = 10              # number of control points per segment 
    true_course       = 80 * Units.degrees
    run_noise_model   = False           # flag to run noise analysis    
    N_lat             = 20              # total number of microphones in longitudinal direction on ground 
    N_long            = 20              # total number of microphones in lateral direction on ground  
    n_lat             = 2               # number of microphones in stencil longitudinal direction on ground 
    n_long            = 2               # number of microphones in stencil lateral direction on ground  
    min_y             = -1E5            # minimum y (lateral) coordinate of acoustic computational domain 
    max_y             = 1E5             # maxiumum y (lateral) coordinate of acoustic computational domain
    min_x             = 0               # minimum x (longitudinal) coordinate of acoustic computational domain 
    max_x             = 2E5             # maxiumum x (longitudinal) coordinate of acoustic computational domain 
    mic_xyz           = None 
    start_x           = 500
    start_y           = 500
    starting_altitude = 0
    ending_altitude   = 100
     
    # SET UP VEHICLE 
    vehicle           = Stopped_Rotor_V2_Vehicle.vehicle_setup() 
    #write(vehicle,"Stopped_Rotor_V2")     
    
    # SET UP CONFIGURATIONS 
    configs           = Stopped_Rotor_V2_Vehicle.configs_setup(vehicle) 
     
    configs_analyses  = Stopped_Rotor_V2_Analyses.analyses_setup(configs,N_lat,N_long,n_lat,n_long,mic_xyz,min_y,max_y,min_x,max_x,
                                                                 start_x,start_y,aircraft_range,run_noise_model)
    
    # SET UP MISSION PROFILE 
    base_mission      = Stopped_Rotor_V2_Missions.full_mission_setup_low_altitude_constant_elevation_cruise(configs_analyses,vehicle,simulated_days,flights_per_day,
                                            aircraft_range,reserve_segment,control_points,recharge_battery,true_course,
                                            starting_altitude,ending_altitude)
    missions_analyses = Stopped_Rotor_V2_Missions.missions_setup(base_mission) 
    
    # DEFINE ANALYSES 
    analyses          = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses 
    
    # FINALIZE SIMULATION 
    configs.finalize()
    analyses.finalize()     
    
    # APPEND MISSION TO SIMULATION 
    mission           = analyses.missions.base
    
    # RUN SIMULATION !!
    noise_results     = mission.evaluate()   
    
    # SAVE RESULTS
    filename          = 'Stopped_Rotor_V2_Full_Mission_Noise_Nx' + str(N_lat) + '_Ny' + str(N_long)
    save_results(noise_results,filename)   
      
    if plot_mission: 
        Stopped_Rotor_V2_Plots.plot_results(noise_results,run_noise_model,save_figure_flag = False )       
                
    tf = time.time() 
    print ('time taken: '+ str(round(((tf-ti)/60),3)) + ' mins')     
    elapsed_range = noise_results.segments[-1].conditions.frames.inertial.position_vector[-1,0] 
    print('Range : ' + str(round(elapsed_range,2)) + ' m  or ' + str(round(elapsed_range/Units.nmi,2)) + ' nmi')
    
    return     

# ----------------------------------------------------------------------
#   Save Results
# ----------------------------------------------------------------------
def save_results(results,filename): 
    pickle_file  = filename + '.pkl'
    with open(pickle_file, 'wb') as file:
        pickle.dump(results, file) 
    return   

# ------------------------------------------------------------------
#   Load Results
# ------------------------------------------------------------------   
def load_results(filename):  
    load_file = filename + '.pkl' 
    with open(load_file, 'rb') as file:
        results = pickle.load(file)
        
    return results

if __name__ == '__main__': 
    main()    
    plt.show()
     
