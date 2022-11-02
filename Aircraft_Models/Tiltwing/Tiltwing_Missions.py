# Tiltwing_Missions.py 

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data  
import numpy as np
import os
import pylab as plt 
import pickle 
import time   

# ------------------------------------------------------------------
#   Full Mission Setup
# ------------------------------------------------------------------  
def full_mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery):
        
  
    starting_elevation = 0*Units.feet
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission     = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport            = SUAVE.Attributes.Airports.Airport() 
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976() 
    mission.airport    = airport    
    atmosphere         = SUAVE.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data          = atmosphere.compute_values(altitude = airport.altitude,temperature_deviation= 1.)    
    
    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment                                             = Segments.Segment() 
    base_segment.state.numerics.number_control_points        = control_points     
    ones_row                                                 = base_segment.state.ones_row
    base_segment.battery_discharge                           = True  
    base_segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
    base_segment.process.finalize.post_process.stability     = SUAVE.Methods.skip    
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  



    # VSTALL Calculation
    m      = vehicle.mass_properties.max_takeoff
    g      = 9.81
    S      = vehicle.reference_area
    atmo   = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    rho    = atmo.compute_values(1000.*Units.feet,0.).density
    CLmax  = 1.2 
    Vstall = float(np.sqrt(2.*m*g/(rho*S*CLmax)))   
  
  
    for day in range(simulated_days):
        print(' ***********  Day ' + str(day) + ' ***********  ')
        for f_idx in range(flights_per_day): 
            flight_no = f_idx + 1        
            # ------------------------------------------------------------------
            #   First Climb Segment: Constant Speed, Constant Rate
            # ------------------------------------------------------------------ 
            segment                                            = Segments.Hover.Climb(base_segment)
            segment.tag                                        = "Vertical_Climb"  + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend(analyses.vertical_climb) 
            segment.altitude_start                             = 0.0  * Units.ft + starting_elevation 
            segment.altitude_end                               = 40.  * Units.ft + starting_elevation 
            segment.climb_rate                                 = 300. * Units['ft/min'] 
            if day == 0:        
                segment.battery_energy                         = vehicle.networks.battery_propeller.battery.max_energy   
            segment.battery_pack_temperature                   = atmo_data.temperature[0,0]             
            segment.state.unknowns.throttle                    = 0.8  * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06) 
            mission.append_segment(segment)  
            
             
            # --------------------------------------------------------------------------
            #   First Transition Segment: Linear Speed, Constant Climb Rate
            # --------------------------------------------------------------------------
            # Use original transition segment, converge on rotor y-axis rotation and throttle
            segment                                             = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
            segment.tag                                         = "Transition_1"  + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend( analyses.vertical_transition_1)
            segment.altitude                                    = 40.0 * Units.ft + starting_elevation 
            segment.acceleration                                = 0.5  * Units['m/s/s']
            segment.air_speed_start                             = 300. * Units['ft/min']        # starts from hover
            segment.air_speed_end                               = 0.4  * Vstall                 # increases linearly in time to stall speed
            segment.pitch_initial                               = 0.0  * Units.degrees  
            segment.pitch_final                                 = 3.6  * Units.degrees   
            segment.state.unknowns.throttle                     = 0.95  * ones_row(1)
            segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
            segment.process.finalize.post_process.stability     = SUAVE.Methods.skip
            segment = vehicle.networks.battery_propeller.add_tiltrotor_transition_unknowns_and_residuals_to_segment(segment,initial_power_coefficient = 0.06)
            # add to misison
            mission.append_segment(segment)
            
            # --------------------------------------------------------------------------
            #   Second Transition Segment: Linear Speed, Linear Climb
            # --------------------------------------------------------------------------
            # Use original transition segment, converge on rotor y-axis rotation and throttle
            segment                                             = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
            segment.tag                                         = "Transition_2"  + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend( analyses.climb_transition)
            segment.altitude_start                              = 40.0 * Units.ft
            segment.altitude_end                                = 500.0 * Units.ft 
            segment.acceleration                                = 0.5  * Units['m/s/s']  
            segment.climb_angle                                 = 7. * Units.deg
            segment.pitch_initial                               = 3.6  * Units.degrees  
            segment.pitch_final                                 = 4.0  * Units.degrees   
            segment.state.unknowns.throttle                     = 0.9  * ones_row(1)
            segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
            segment.process.finalize.post_process.stability     = SUAVE.Methods.skip
            segment = vehicle.networks.battery_propeller.add_tiltrotor_transition_unknowns_and_residuals_to_segment(segment, 
                                                                                                                    initial_power_coefficient = 0.03)
            # add to misison
            mission.append_segment(segment) 
            

            # --------------------------------------------------------------------------
            # Third Transition Segment: Linear Speed, Linear Climb
            # --------------------------------------------------------------------------
            # Use original transition segment, converge on rotor y-axis rotation and throttle
            segment                                             = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
            segment.tag                                         = "Transition_3"  + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend( analyses.climb_transition)
            segment.altitude_start                              = 500.0 * Units.ft
            segment.altitude_end                                = 1000.0 * Units.ft 
            segment.acceleration                                = 0.25  * Units['m/s/s'] 
            segment.climb_angle                                 = 5. * Units.deg
            segment.pitch_initial                               = 4.0  * Units.degrees  
            segment.pitch_final                                 = 5.0  * Units.degrees   
            segment.state.unknowns.throttle                     = 0.8  * ones_row(1)
            segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
            segment.process.finalize.post_process.stability     = SUAVE.Methods.skip
            segment = vehicle.networks.battery_propeller.add_tiltrotor_transition_unknowns_and_residuals_to_segment(segment, 
                                                                                                                    initial_power_coefficient = 0.06)
            # add to misison
            mission.append_segment(segment)   
                             
            # ------------------------------------------------------------------
            #   First Cruise Segment: Constant Acceleration, Constant Altitude
            # ------------------------------------------------------------------ 
            segment                           = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            segment.tag                       = "Climb" + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend(analyses.climb) 
            segment.climb_rate                = 500. * Units['ft/min']
            segment.air_speed_start           = 107.5  * Units['mph']
            segment.air_speed_end             = 175.   * Units['mph'] 
            segment.altitude_start            = 1000.0 * Units.ft  + starting_elevation                    
            segment.altitude_end              = 2500.0 * Units.ft                
            segment.state.unknowns.throttle   = 0.80 * ones_row(1) 
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03)     
            mission.append_segment(segment)     
        
            # ------------------------------------------------------------------
            #   First Cruise Segment: Constant Acceleration, Constant Altitude
            # ------------------------------------------------------------------ 
            segment                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
            segment.tag                      = "Cruise" + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend(analyses.cruise) 
            segment.altitude                 = 2500.0 * Units.ft               
            segment.air_speed                = 175.   * Units['mph'] 
            cruise_distance                  = aircraft_range  - 22.62 * Units.nmi
            segment.distance                 = cruise_distance
            segment.state.unknowns.throttle  = 0.8 * ones_row(1)   
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.01)     
            mission.append_segment(segment)     
            
            # ------------------------------------------------------------------
            #    Descent Segment: Constant Acceleration, Constant Altitude
            # ------------------------------------------------------------------ 
            segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            segment.tag                      = "Descent"  + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend(analyses.climb)
            segment.climb_rate               = -300. * Units['ft/min']
            segment.air_speed_start          = 175.   * Units['mph']
            segment.air_speed_end            = 100.   * Units['mph'] 
            segment.altitude_start           = 2500.0 * Units.ft 
            segment.altitude_end             = 500.0 * Units.ft + starting_elevation      
            segment.state.unknowns.throttle  = 0.6 * ones_row(1) 
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
            mission.append_segment(segment)     
            
            if reserve_segment: 
                # ------------------------------------------------------------------
                #   Reserve Climb Segment 
                # ------------------------------------------------------------------ 
                segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                segment.tag                      = "Reserve_Climb"  + "_F_" + str(flight_no) + "_D" + str (day)
                segment.analyses.extend(analyses.climb) 
                segment.climb_rate               = 500. * Units['ft/min']
                segment.air_speed_start          = 100.   * Units['mph'] 
                segment.air_speed_end            = 150.   * Units['mph'] 
                segment.altitude_start           = 100.0 * Units.ft+ starting_elevation 
                segment.altitude_end             = 1000.0 * Units.ft              
                segment.state.unknowns.throttle  = 0.8 * ones_row(1) 
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
                mission.append_segment(segment)      
            
                # ------------------------------------------------------------------
                #   First Cruise Segment: Constant Acceleration, Constant Altitude
                # ------------------------------------------------------------------ 
                segment                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment) 
                segment.tag                      = "Reserve_Cruise"  + "_F_" + str(flight_no) + "_D" + str (day)
                segment.analyses.extend(analyses.cruise)  
                segment.air_speed                = 150.   * Units['mph'] 
                segment.distance                 = cruise_distance*0.1    
                segment.state.unknowns.throttle  = 0.8 * ones_row(1) 
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.01)  
                mission.append_segment(segment)     
            
                # ------------------------------------------------------------------
                #   Reserve Descent Segment: Constant Acceleration, Constant Altitude
                # ------------------------------------------------------------------ 
                segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                segment.tag                      = "Reserve_Descent"  + "_F_" + str(flight_no) + "_D" + str (day)
                segment.analyses.extend(analyses.climb)
                segment.climb_rate               = -300. * Units['ft/min']
                segment.air_speed_start          = 150.   * Units['mph']
                segment.air_speed_end            = 100.   * Units['mph']
                segment.altitude_start           = 1000.0 * Units.ft
                segment.altitude_end             = 100.0 * Units.ft    + starting_elevation                
                segment.state.unknowns.throttle  = 0.6 * ones_row(1) 
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
                mission.append_segment(segment)        
             
            # --------------------------------------------------------------------------
            # Decelerating Transition Segment: Linear Speed, Constant Climb Rate
            # --------------------------------------------------------------------------
            # Use original transition segment, converge on rotor y-axis rotation and throttle
            segment                                             = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
            segment.tag                                         = "Transition_4"  + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend( analyses.climb_transition)
            segment.altitude_start                              = 500.0 * Units.ft    + starting_elevation     
            segment.altitude_end                                = 40.0 * Units.ft    + starting_elevation     
            segment.acceleration                                = -0.35  * Units['m/s/s'] 
            segment.climb_angle                                 = -3. * Units.deg
            segment.pitch_initial                               = 4.0  * Units.degrees  
            segment.pitch_final                                 = 0.0  * Units.degrees   
            segment.state.unknowns.throttle                     = 0.28  * ones_row(1)
            segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
            segment.process.finalize.post_process.stability     = SUAVE.Methods.skip
            segment = vehicle.networks.battery_propeller.add_tiltrotor_transition_unknowns_and_residuals_to_segment(segment, 
                                                                                                                    initial_power_coefficient = 0.06)
            # add to misison
            mission.append_segment(segment)   
        
            # --------------------------------------------------------------------------
            #   Fifth Transition Segment: Linear Speed, Constant Climb Rate
            # --------------------------------------------------------------------------
            # Use original transition segment, converge on rotor y-axis rotation and throttle
            segment                                             = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
            segment.tag                                         = "Transition_5"  + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend( analyses.vertical_transition_1)
            segment.altitude                                    = 40.0 * Units.ft + starting_elevation 
            segment.acceleration                                = -0.5  * Units['m/s/s']
            segment.air_speed_start                             = 20.   * Units['mph'] 
            segment.air_speed_end                               = 300. * Units['ft/min']         
            segment.pitch_initial                               = 3.6  * Units.degrees  
            segment.pitch_final                                 = 0.0  * Units.degrees   
            segment.state.unknowns.throttle                     = 0.95  * ones_row(1)
            segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
            segment.process.finalize.post_process.stability     = SUAVE.Methods.skip
            segment = vehicle.networks.battery_propeller.add_tiltrotor_transition_unknowns_and_residuals_to_segment(segment,initial_power_coefficient = 0.06)
            # add to misison
            mission.append_segment(segment)
            
            # ------------------------------------------------------------------
            #   Descent Segment: Constant Speed, Constant Rate
            # ------------------------------------------------------------------ 
            segment                          = Segments.Hover.Descent(base_segment)
            segment.tag                      = "Vertical_Descent"  + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend( analyses.vertical_descent) 
            segment.altitude_start           = 40.0  * Units.ft + starting_elevation 
            segment.altitude_end             = 0.  * Units.ft + starting_elevation 
            segment.descent_rate             = 300. * Units['ft/min']  
            segment.state.unknowns.throttle  = 0.6  * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06)  
            mission.append_segment(segment)  
            
            if recharge_battery:
                # ------------------------------------------------------------------
                #  Charge Segment: 
                # ------------------------------------------------------------------  
                # Charge Model 
                segment                                                  = Segments.Ground.Battery_Charge_Discharge(base_segment)     
                segment.tag                                              = 'Charge Day ' + "_F_" + str(flight_no) + "_D" + str (day)  
                segment.analyses.extend(analyses.base)           
                segment.battery_discharge                                = False   
                if flight_no  == flights_per_day:  
                    segment.increment_battery_cycle_day=True                     
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment )    
                mission.append_segment(segment)   

    return mission 

# ------------------------------------------------------------------
#   Noise (Approach/Departure) Mission Setup
# ------------------------------------------------------------------     
def approach_departure_mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery):
      
    starting_elevation = 0*Units.feet
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission     = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport            = SUAVE.Attributes.Airports.Airport() 
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976() 
    mission.airport    = airport   

    atmosphere    = SUAVE.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data          = atmosphere.compute_values(altitude = airport.altitude,temperature_deviation= 1.)    
    
    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment                                             = Segments.Segment() 
    base_segment.state.numerics.number_control_points        = control_points     
    ones_row                                                 = base_segment.state.ones_row
    base_segment.battery_discharge                           = True  
    base_segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
    base_segment.process.finalize.post_process.stability     = SUAVE.Methods.skip    
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health   
 

    # VSTALL Calculation
    m      = vehicle.mass_properties.max_takeoff
    g      = 9.81
    S      = vehicle.reference_area
    atmo   = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    rho    = atmo.compute_values(1000.*Units.feet,0.).density
    CLmax  = 1.2 
    Vstall = float(np.sqrt(2.*m*g/(rho*S*CLmax)))   
    
    
    
     
     
    # --------------------------------------------------------------------------
    # Decelerating Transition Segment: Linear Speed, Constant Climb Rate
    # --------------------------------------------------------------------------
    # Use original transition segment, converge on rotor y-axis rotation and throttle
    segment                                             = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
    segment.tag                                         = "Transition_4"  
    segment.analyses.extend( analyses.climb_transition)
    segment.air_speed                                   = 100.   * Units['mph'] 
    segment.altitude_start                              = 500.0 * Units.ft    + starting_elevation     
    segment.altitude_end                                = 40.0 * Units.ft    + starting_elevation     
    segment.acceleration                                = -0.35  * Units['m/s/s'] 
    segment.climb_angle                                 = -3. * Units.deg
    segment.pitch_initial                               = 4.0  * Units.degrees  
    segment.pitch_final                                 = 0.0  * Units.degrees   
    segment.state.unknowns.throttle                     = 0.28  * ones_row(1) 
    segment.battery_energy                              = vehicle.networks.battery_propeller.battery.max_energy*0.7 
    segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability     = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_tiltrotor_transition_unknowns_and_residuals_to_segment(segment, 
                                                                                                            initial_power_coefficient = 0.06)
    # add to misison
    mission.append_segment(segment)   

    # --------------------------------------------------------------------------
    #   Fifth Transition Segment: Linear Speed, Constant Climb Rate
    # --------------------------------------------------------------------------
    # Use original transition segment, converge on rotor y-axis rotation and throttle
    segment                                             = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
    segment.tag                                         = "Transition_5"   
    segment.analyses.extend( analyses.vertical_transition_1)
    segment.altitude                                    = 40.0 * Units.ft + starting_elevation 
    segment.acceleration                                = -0.5  * Units['m/s/s']
    segment.air_speed_start                             = 20.   * Units['mph'] 
    segment.air_speed_end                               = 300. * Units['ft/min']         
    segment.pitch_initial                               = 3.6  * Units.degrees  
    segment.pitch_final                                 = 0.0  * Units.degrees   
    segment.state.unknowns.throttle                     = 0.95  * ones_row(1)
    segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability     = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_tiltrotor_transition_unknowns_and_residuals_to_segment(segment,initial_power_coefficient = 0.06)
    # add to misison
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                          = Segments.Hover.Descent(base_segment)
    segment.tag                      = "Vertical_Descent"   
    segment.analyses.extend( analyses.vertical_descent) 
    segment.altitude_start           = 40.0  * Units.ft + starting_elevation 
    segment.altitude_end             = 0.  * Units.ft + starting_elevation 
    segment.descent_rate             = 300. * Units['ft/min']  
    segment.state.unknowns.throttle  = 0.6  * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06)  
    mission.append_segment(segment)  
    
    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                                            = Segments.Hover.Climb(base_segment)
    segment.tag                                        = "Vertical_Climb"   
    segment.analyses.extend(analyses.vertical_climb) 
    segment.altitude_start                             = 0.0  * Units.ft + starting_elevation 
    segment.altitude_end                               = 40.  * Units.ft + starting_elevation 
    segment.climb_rate                                 = 300. * Units['ft/min']   
    segment.battery_pack_temperature                   = atmo_data.temperature[0,0]             
    segment.state.unknowns.throttle                    = 0.8  * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06) 
    mission.append_segment(segment)  
    
     
    # --------------------------------------------------------------------------
    #   First Transition Segment: Linear Speed, Constant Climb Rate
    # --------------------------------------------------------------------------
    # Use original transition segment, converge on rotor y-axis rotation and throttle
    segment                                             = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
    segment.tag                                         = "Transition_1"   
    segment.analyses.extend( analyses.vertical_transition_1)
    segment.altitude                                    = 40.0 * Units.ft + starting_elevation 
    segment.acceleration                                = 0.5  * Units['m/s/s']
    segment.air_speed_start                             = 300. * Units['ft/min']        # starts from hover
    segment.air_speed_end                               = 0.4  * Vstall                 # increases linearly in time to stall speed
    segment.pitch_initial                               = 0.0  * Units.degrees  
    segment.pitch_final                                 = 3.6  * Units.degrees   
    segment.state.unknowns.throttle                     = 0.95  * ones_row(1)
    segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability     = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_tiltrotor_transition_unknowns_and_residuals_to_segment(segment,initial_power_coefficient = 0.06)
    # add to misison
    mission.append_segment(segment)
    
    # --------------------------------------------------------------------------
    #   Second Transition Segment: Linear Speed, Linear Climb
    # --------------------------------------------------------------------------
    # Use original transition segment, converge on rotor y-axis rotation and throttle
    segment                                             = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
    segment.tag                                         = "Transition_2"  
    segment.analyses.extend( analyses.climb_transition)
    segment.altitude_start                              = 40.0 * Units.ft
    segment.altitude_end                                = 500.0 * Units.ft 
    segment.acceleration                                = 0.5  * Units['m/s/s']  
    segment.climb_angle                                 = 7. * Units.deg
    segment.pitch_initial                               = 3.6  * Units.degrees  
    segment.pitch_final                                 = 4.0  * Units.degrees   
    segment.state.unknowns.throttle                     = 0.9  * ones_row(1)
    segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability     = SUAVE.Methods.skip
    segment = vehicle.networks.battery_propeller.add_tiltrotor_transition_unknowns_and_residuals_to_segment(segment, 
                                                                                                            initial_power_coefficient = 0.03)
    # add to misison
    mission.append_segment(segment) 
        
    
    return mission

# ------------------------------------------------------------------
#  Hover Noise Mission Setup
# ------------------------------------------------------------------   
def hover_mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery):
         
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission     = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport            = SUAVE.Attributes.Airports.Airport() 
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976() 
    mission.airport    = airport   

    atmosphere    = SUAVE.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data          = atmosphere.compute_values(altitude = airport.altitude,temperature_deviation= 1.)    
    
    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment                                             = Segments.Segment() 
    base_segment.state.numerics.number_control_points        = 3     
    ones_row                                                 = base_segment.state.ones_row
    base_segment.battery_discharge                           = True  
    base_segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
    base_segment.process.finalize.post_process.stability     = SUAVE.Methods.skip    
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  

 
    segment                                            = Segments.Hover.Climb(base_segment)
    segment.tag                                        = "Hover" # very small climb so that broadband noise does not return 0's  
    segment.analyses.extend(analyses.vertical_climb) 
    segment.altitude_start                             = 500.0  * Units.ft  
    segment.altitude_end                               = 500.1  * Units.ft 
    segment.climb_rate                                 = 100. * Units['ft/min']  
    segment.battery_energy                             = vehicle.networks.battery_propeller.battery.max_energy   
    segment.battery_pack_temperature                   = atmo_data.temperature[0,0]             
    segment.state.unknowns.throttle                    = 0.6  * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06) 
    mission.append_segment(segment)  
 

    return mission

# ------------------------------------------------------------------
#   Missions Setup
# ------------------------------------------------------------------   
def missions_setup(base_mission):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission


    # done!
    return missions   