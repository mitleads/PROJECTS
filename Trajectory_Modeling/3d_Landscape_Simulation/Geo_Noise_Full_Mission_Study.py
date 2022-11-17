# Stopped_Rotor_Touch_Go_Simulation.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data    
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools.generate_microphone_points import preprocess_topography_and_route_data
import pickle
from SUAVE.Plots.Performance.Mission_Plots import *  
from SUAVE.Plots.Geometry                  import * 

import time  
import numpy as np
import pylab as plt

import sys 
sys.path.append('../../Aircraft_Models/Stopped_Rotor_V2')  

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
    ti                         = time.time()
    RUN_NEW_MODEL_FLAG         = False
    
    # -------------------------------------------------------------------------------------------    
    # SET UP SIMULATION PARAMETERS   
    # -------------------------------------------------------------------------------------------  
    simulated_days             = 1               # number of days simulated 
    flights_per_day            = 1               # number of flights per day   
    recharge_battery           = False           # flag to simulate battery recharge  
    plot_mission               = True            # plot mission flag  
    control_points             = 10              # number of control points per segment 
    
    # noise analysis parameters 
    run_noise_model            = True     
    
    # strings for savign results 
    city                       = 'LA'  
    departure_location         = 'LAX'   
    destination_location       = 'DIS'  
    

    if RUN_NEW_MODEL_FLAG:    
        topography_data = preprocess_topography_and_route_data(topography_file                       = 'LA_Metropolitan_Zoomed_Area.txt',
                                                               departure_coordinates                 = [33.94067953101678, -118.40513722978149],
                                                               destination_coordinates               = [33.81713622114423, -117.92111163722772],
                                                               number_of_latitudinal_microphones     = 201,
                                                               number_of_longitudinal_microphones    = 101,
                                                               latitudinal_microphone_stencil_size   = 3,
                                                               longitudinal_microphone_stencil_size  = 3 )
         
        # -------------------------------------------------------------------------------------------    
        # SET UP VEHICLE
        # ------------------------------------------------------------------------------------------- 
        vehicle           = Stopped_Rotor_V2_Vehicle.vehicle_setup()  
    
        # -------------------------------------------------------------------------------------------    
        # SET UP CONFIGURATIONS 
        # -------------------------------------------------------------------------------------------
        configs           = Stopped_Rotor_V2_Vehicle.configs_setup(vehicle)  
        configs_analyses  = Stopped_Rotor_V2_Analyses.topography_analyses_setup(configs,topography_data,run_noise_model)
    
        # -------------------------------------------------------------------------------------------    
        # SET UP MISSION PROFILE  
        # -------------------------------------------------------------------------------------------    
        base_mission      = Stopped_Rotor_V2_Missions.low_altitude_constant_elevation_cruise(configs_analyses,vehicle,simulated_days,flights_per_day,control_points,recharge_battery,topography_data)
        missions_analyses = Stopped_Rotor_V2_Missions.missions_setup(base_mission) 
    
        # -------------------------------------------------------------------------------------------    
        # DEFINE ANALYSES 
        # -------------------------------------------------------------------------------------------
        analyses          = SUAVE.Analyses.Analysis.Container()
        analyses.configs  = configs_analyses
        analyses.missions = missions_analyses 
        
    
        # -------------------------------------------------------------------------------------------    
        # FINALIZE SIMULATION 
        # -------------------------------------------------------------------------------------------    
        configs.finalize()
        analyses.finalize()   
    
        # -------------------------------------------------------------------------------------------    
        # APPEND MISSION TO SIMULATION 
        # -------------------------------------------------------------------------------------------    
        mission           = analyses.missions.base
        
    
        # -------------------------------------------------------------------------------------------    
        # RUN SIMULATION !!
        # -------------------------------------------------------------------------------------------
        noise_results     = mission.evaluate() 
    
        # -------------------------------------------------------------------------------------------    
        # SAVE RESULTS
        # -------------------------------------------------------------------------------------------
        filename          = 'SR_V2_Geo_Noise' + city + '_' +  departure_location + '_to_' + destination_location
        save_results(noise_results,filename)   
    
    else:
        filename          = 'SR_V2_Geo_Noise' + city + '_' +  departure_location + '_to_' + destination_location
        noise_results = load_results(filename) 
        
    if plot_mission: 
        Stopped_Rotor_V2_Plots.plot_results(noise_results,run_noise_model,save_figure_flag = True)       
    
    
    tf = time.time() 
    print ('time taken: '+ str(round(((tf-ti)/60),3)) + ' mins')   
    
    elapsed_range = noise_results.segments[-1].conditions.frames.inertial.position_vector[-1,0] 
    print('Range : ' + str(round(elapsed_range,2)) + ' m  or ' + str(round(elapsed_range/Units.nmi,2)) + ' nmi')   
        
    return 


# ----------------------------------------------------------------------
#   Save Results
# ----------------------------------------------------------------------
def save_results(results,filename): 
    pickle_file  =  filename + '.pkl'
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
     
