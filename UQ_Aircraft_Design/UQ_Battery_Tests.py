# UQ Test Simulations 

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
import matplotlib.pyplot as plt 
import time 

from NMC_Battery_Cell import full_setup

def main():    
    
    ti                    = time.time() 
    
    # Create the distributions around zero mean  
    c_rate              = 1
    capacity_dist       = cp.Normal(3.55 , 0.1)    # values of battery capacity           (electrical)
    cp_dist             = cp.Normal(1100 , 100)    # values of specific heat capacity     (thermal) 
    V0_dist             = cp.Normal(3.6  , 0.1)     # values of nominal voltage         (electrical) 
    delta_R0_dist       = cp.Normal(0, 0.01)    # delta for resistance function     (electrical)  

    # Define the parameter dictionary 
    parameters = {"c_rate": c_rate,
                  "cycles_per_day": 4,
                  "days"  : 100,
                  "E"     : capacity_dist,
                  "c_p"   : cp_dist,
                  "V_0"   : V0_dist,
                  "R_0"   : delta_R0_dist}
    
    UQ_Res = {} 
  

    # Create a model from the battery_cell_simulation function and add labels 
    #E_model = un.Model(run=single_discharge_battery_temperature_PCE, labels=["time (hr)", "Temperature ($\degree$C)"])    
    #V_model = un.Model(run=single_discharge_battery_voltage_PCE, labels=["time (hr)", "V (V)"])  
    E_model = un.Model(run=extended_discharge_battery_capacity_fade_PCE, labels=["time (hr)", "E/E_0"])    
    #V_model = un.Model(run=extended_discharge_battery_resistance_growth_PCE, labels=["time (hr)", "R/R_0"])  
    
    # Set up the uncertainty quantification
    E_UQ = un.UncertaintyQuantification(model= E_model, parameters=parameters) 
    #V_UQ = un.UncertaintyQuantification(model= V_model, parameters=parameters) 
    
    # Perform the uncertainty quantification using polynomial chaos with point collocatio  
    UQ_Res['Energy']  = E_UQ.quantify(seed=10,polynomial_order=4,nr_pc_mc_samples=10**3)
    #UQ_Res['Voltage'] = V_UQ.quantify(seed=10,polynomial_order=4,nr_pc_mc_samples=10**3) 
    
    plot_uq_results(UQ_Res)  

    tf = time.time()
    dt = tf-ti
    print('Time Elapsed ' + str(dt/60) + ' min')        
    
    return 

# --------------------------------------------------------------
# Singe Dischage Battery Energy Tests 
# --------------------------------------------------------------
def single_discharge_battery_energy_PCE(c_rate,cycles_per_day,days,E,c_p,V_0,R_0):  
    results             = discharge_simulation(c_rate,cycles_per_day,days,E,c_p,V_0,R_0)
    elapsed_time        = results.segments[0].conditions.frames.inertial.time[:,0]/3600                  
    battery_energy      = (results.segments[0].conditions.propulsion.battery_energy[:,0]/Units.Wh) 
    
    return elapsed_time,battery_energy  
 

def single_discharge_battery_temperature_PCE(c_rate,cycles_per_day,days,E,c_p,V_0,R_0):  
    results             = discharge_simulation(c_rate,cycles_per_day,days,E,c_p,V_0,R_0)
    elapsed_time        = results.segments[0].conditions.frames.inertial.time[:,0]/3600                
    battery_temperature = results.segments[0].conditions.propulsion.battery_cell_temperature[:,0] - 273  
    
    return elapsed_time,battery_temperature 


def single_discharge_battery_voltage_PCE(c_rate,cycles_per_day,days,E,c_p,V_0,R_0):  
    results             = discharge_simulation(c_rate,cycles_per_day,days,E,c_p,V_0,R_0)
    elapsed_time        = results.segments[0].conditions.frames.inertial.time[:,0]/3600                 
    voltage             = results.segments[0].conditions.propulsion.battery_voltage_under_load[:,0]
    
    return elapsed_time,voltage

 

# --------------------------------------------------------------
# Extended Dischage Battery Energy Tests 
# --------------------------------------------------------------

def extened_discharge_battery_energy_PCE(c_rate,cycles_per_day,days,E,c_p,V_0,R_0):  
    results             = discharge_simulation(c_rate,cycles_per_day,days,E,c_p,V_0,R_0)
    
    return results.Time,results.energy 

def extended_discharge_battery_capacity_fade_PCE(c_rate,cycles_per_day,days,E,c_p,V_0,R_0):  
    results             = discharge_simulation(c_rate,cycles_per_day,days,E,c_p,V_0,R_0)
    return results.Time,results.normalized_max_energy


def extended_discharge_battery_resistance_growth_PCE(c_rate,cycles_per_day,days,E,c_p,V_0,R_0):  
    results             = discharge_simulation(c_rate,cycles_per_day,days,E,c_p,V_0,R_0)
    
    return results.Time,results.normalized_R_0


def discharge_simulation(c_rate,cycles_per_day,days,E,c_p,V_0,R_0):

    # define uncertainties 
    uncertainties              = Data()  
    uncertainties.V0           = V_0
    uncertainties.R0           = R_0  
    uncertainties.Cp           = c_p  
    uncertainties.E            = E  
    
    configs, analyses = full_setup(c_rate,uncertainties,cycles_per_day,days)
    analyses.finalize()     
    mission = analyses.missions.base
    results = mission.evaluate()  
    

    Results                            = Data()
    
    if days == 0 and cycles_per_day == 0: 
        Results.Time        = results.segments[0].conditions.frames.inertial.time[:,0]/3600                 
        Results.volts       = results.segments[0].conditions.propulsion.battery_voltage_under_load[:,0]    
        Results.temp        = results.segments[0].conditions.propulsion.battery_cell_temperature[:,0] - 273   
        Results.energy      = (results.segments[0].conditions.propulsion.battery_energy[:,0]/Units.Wh)  
    
    else: 
        Results.Time                       = np.zeros((days))
        Results.energy                     = np.zeros_like(Results.Time)  
        Results.current                    = np.zeros_like(Results.Time)
        Results.specific_power             = np.zeros_like(Results.Time) 
        Results.SOC                        = np.zeros_like(Results.Time)
        Results.volts                      = np.zeros_like(Results.Time)
        Results.volts_oc                   = np.zeros_like(Results.Time) 
        Results.battery_amp_hr             = np.zeros_like(Results.Time)
        Results.C_rate                     = np.zeros_like(Results.Time)
        Results.temp                       = np.zeros_like(Results.Time)   
        Results.charge_throughput          = np.zeros_like(Results.Time)  
        Results.normalized_max_energy      = np.zeros_like(Results.Time)  
        Results.normalized_R_0             = np.zeros_like(Results.Time)   
        
        k = 0
        i = 0  
        for segment in results.segments.values(): 
            if k  % (cycles_per_day*2) == 0:
                Results.Time[i]                    = segment.conditions.frames.inertial.time[0,0] / Units.hr 
                Results.energy[i]                  = segment.conditions.propulsion.battery_energy[0,0] /Units.Wh
                Results.current[i]                 = segment.conditions.propulsion.battery_current[0,0] 
                Results.specific_power[i]          = segment.conditions.propulsion.battery_specfic_power[0,0] 
                Results.SOC[i]                     = segment.conditions.propulsion.battery_state_of_charge[0,0]
                Results.volts[i]                   = segment.conditions.propulsion.battery_voltage_under_load[0,0] 
                Results.volts_oc[i]                = segment.conditions.propulsion.battery_voltage_open_circuit[0,0]  
                Results.temp[i]                    = segment.conditions.propulsion.battery_cell_temperature[0,0]         
                Results.normalized_max_energy[i]   = segment.conditions.propulsion.battery_capacity_fade_factor  
                Results.normalized_R_0[i]          = segment.conditions.propulsion.battery_resistance_growth_factor  
                i+= 1
            k+= 1
        
    return Results 

if __name__ == '__main__':
    main()
    plt.show()
     