# Stopped_Rotor_Touch_Go_Simulation.py
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
from SUAVE.Plots.Geometry                  import * 

import time 
import numpy as np
import pylab as plt

import Stopped_Rotor_Vehicle
import Stopped_Rotor_Analyses 
import Stopped_Rotor_Missions
import Stopped_Rotor_Plots 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main(): 

    # start simulation clock
    ti                = time.time() 
    
    # SET UP SIMULATION PARAMETERS     
    simulated_days    = 1               # number of days simulated 
    flights_per_day   = 1               # number of flights per day 
    aircraft_range    = 55 *Units.nmi   # total ground distance 
    reserve_segment   = False           # reserve segment flag, if true, ground distance not included  
    recharge_battery  = False           # flag to simulate battery recharge  
    plot_mission      = True            # plot mission flag  
    control_points    = 10              # number of control points per segment 
    run_noise_model   = False           # flag to run noise analysis    
    N_gm_x            = 10              # number of microphones in longitudinal direction on ground 
    N_gm_y            = 5               # number of microphones in lateral direction on ground  
    min_y             = 1E-3            # minimum y (lateral) coordinate of acoustic computational domain 
    max_y             = 0.5*Units.nmi   # maxiumum y (lateral) coordinate of acoustic computational domain
    min_x             = 0.*Units.nmi    # minimum x (longitudinal) coordinate of acoustic computational domain 
    max_x             = 5.79*Units.nm  # maxiumum x (longitudinal) coordinate of acoustic computational domain 
    
    
    # SET UP VEHICLE 
    vehicle           = Stopped_Rotor_Vehicle.vehicle_setup() 
    
    # SET UP CONFIGURATIONS 
    configs           = Stopped_Rotor_Vehicle.configs_setup(vehicle) 
    
    # devide computational domain to increase noise prediction
    if run_noise_model:
        n = 3  
    else:
        n = 2
    Y_LIM = np.linspace(min_y,max_y,n)     
    X_LIM = np.linspace(min_x,max_x,n)        
    Q_idx             = 1
    for i in range(len(X_LIM)-1):
        for j in range(len(Y_LIM)-1): 
            print('Running Quardant:' + str(Q_idx)) 
            min_x = X_LIM[i]
            max_x = X_LIM[i+1]
            min_y = Y_LIM[j]
            max_y = Y_LIM[j+1]
        
            configs_analyses  = Stopped_Rotor_Analyses.analyses_setup(configs,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,
                                               aircraft_range,run_noise_model) 
            
            # SET UP MISSION PROFILE 
            base_mission      = Stopped_Rotor_Missions.approach_departure_mission_setup(configs_analyses,vehicle,simulated_days,flights_per_day,
                                                    aircraft_range,reserve_segment,control_points,recharge_battery )
            missions_analyses = Stopped_Rotor_Missions.missions_setup(base_mission) 
            
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
            filename          = 'Stopped_Rotor_Approach_Departure_Noise_Q' + str(Q_idx)+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
            save_results(noise_results,filename)  
            
            # stop simulation clock
            Q_idx += 1   
      
    if plot_mission: 
        Stopped_Rotor_Plots.plot_results(noise_results,run_noise_model)       
                
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
     
