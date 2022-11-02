# Stopped_Rotor_Missions.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units    
import numpy as np 

# ------------------------------------------------------------------
#   Full Mission Setup
# ------------------------------------------------------------------
def full_mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range,
                       reserve_segment,control_points,recharge_battery): 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # airport
    airport           = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976() 
    mission.airport    = airport      

    atmosphere         = SUAVE.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data          = atmosphere.compute_values(altitude = airport.altitude,temperature_deviation= 1.)     

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment           
    base_segment                                             = Segments.Segment() 
    base_segment.battery_discharge                           = True  
    base_segment.state.numerics.number_control_points        = control_points
    ones_row                                                 = base_segment.state.ones_row
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  
    base_segment.process.finalize.post_process.stability  = SUAVE.Methods.skip


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
            ## ------------------------------------------------------------------
            ##   First Climb Segment: Constant Speed, Constant Rate
            ## ------------------------------------------------------------------
            #segment     = Segments.Hover.Climb(base_segment)
            #segment.tag = "Vertical_Climb"  + "_F_" + str(flight_no) + "_D" + str (day) 
            #segment.analyses.extend( analyses.base )  
            #segment.altitude_start                          = 0.0  * Units.ft  
            #segment.altitude_end                            = 40.  * Units.ft  
            #segment.climb_rate                              = 500. * Units['ft/min'] 
            #if day == 0:        
                #segment.battery_energy                         = vehicle.networks.lift_cruise.battery.max_energy   
            #segment.battery_pack_temperature                   = atmo_data.temperature[0,0]             
            #segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
            #segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
            #segment.process.finalize.post_process.stability = SUAVE.Methods.skip
            #segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment,\
                                                                                    #initial_lift_rotor_power_coefficient = 0.02,
                                                                                    #initial_throttle_lift = 0.7)
            #mission.append_segment(segment)
            
            ## ------------------------------------------------------------------
            ##   First Cruise Segment: Transition
            ## ------------------------------------------------------------------
         
            #segment                                    = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
            #segment.tag                                = "Vertical_Transition" + "_F_" + str(flight_no) + "_D" + str (day) 
            #segment.analyses.extend( analyses.transition ) 
            #segment.altitude                           = 40.  * Units.ft                 
            #segment.air_speed_start                    = 500. * Units['ft/min']
            #segment.air_speed_end                      = 0.8 * Vstall
            #segment.acceleration                       = 2.5
            #segment.pitch_initial                      = 0.0 * Units.degrees
            #segment.pitch_final                        = 2.  * Units.degrees  
            #segment.state.unknowns.throttle            = 0.70    *  ones_row(1) 
            #segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
            #segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
            #segment.process.finalize.post_process.stability = SUAVE.Methods.skip
            #segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment,  
                                                                 #initial_throttle_lift = 0.9) 
            #mission.append_segment(segment)
            
            ## ------------------------------------------------------------------
            ##   First Cruise Segment: Transition
            ## ------------------------------------------------------------------ 
            #segment                                         = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
            #segment.tag                                     = "Climb_Transition"+ "_F_" + str(flight_no) + "_D" + str (day) 
            #segment.analyses.extend( analyses.base )    
            #segment.altitude_start                          = 40.0 * Units.ft  
            #segment.altitude_end                            = 100.0 * Units.ft  
            #segment.air_speed                               = 0.8   * Vstall
            #segment.climb_angle                             = 2     * Units.degrees   
            #segment.acceleration                            = 0.25   * Units['m/s/s']    
            #segment.pitch_initial                           = 7.    * Units.degrees  
            #segment.pitch_final                             = 5.    * Units.degrees   
            #segment.state.unknowns.throttle                 = 0.80   *  ones_row(1)
            #segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
            #segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
            #segment.process.finalize.post_process.stability = SUAVE.Methods.skip 
            #segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment, 
                                                                 #initial_throttle_lift = 0.8) 
            #mission.append_segment(segment) 
          
            ## ------------------------------------------------------------------
            ##   Second Climb Segment: Constant Speed, Constant Rate
            ## ------------------------------------------------------------------ 
            #segment                                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            #segment.tag                                      = "Climb_1"   + "_F_" + str(flight_no) + "_D" + str (day) 
            #segment.analyses.extend( analyses.base ) 
            #segment.altitude_start                           = 100.0 * Units.ft  
            #segment.altitude_end                             = 500. * Units.ft  
            #segment.climb_rate                               = 600.  * Units['ft/min']
            #segment.air_speed_start                          = 95.   * Units['mph']
            #segment.air_speed_end                            = 125.  * Units['mph']  
            #segment.state.unknowns.throttle                  = 0.9   *  ones_row(1)  
            #segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)   
            #mission.append_segment(segment)  
        
            ## ------------------------------------------------------------------
            ##   Third Climb Segment: Constant Acceleration, Constant Rate
            ## ------------------------------------------------------------------ 
            #segment                                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            #segment.tag                                      = "Climb_2" + "_F_" + str(flight_no) + "_D" + str (day) 
            #segment.analyses.extend( analyses.base ) 
            #segment.altitude_start                           = 500.0 * Units.ft  
            #segment.altitude_end                             = 2500. * Units.ft
            #segment.climb_rate                               = 500.  * Units['ft/min']
            #segment.air_speed_start                          = 125.  * Units['mph']  
            #segment.air_speed_end                            = 175.  * Units['mph']    
            #segment.state.unknowns.throttle                  = 0.90    *  ones_row(1)
            #segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)    
            #mission.append_segment(segment)  
        
            # ------------------------------------------------------------------
            #   Third Cruise Segment: Constant Acceleration, Constant Altitude
            # ------------------------------------------------------------------ 
            segment                                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
            segment.tag                                      = "Cruise" + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend( analyses.base )                 
            segment.battery_energy                         = vehicle.networks.lift_cruise.battery.max_energy   
            segment.altitude                                 = 2500.0 * Units.ft
            segment.air_speed                                = 175.   * Units['mph']
            cruise_distance                                  = aircraft_range - 32.26 * Units.nmi    
            segment.distance                                 = cruise_distance  
            segment.state.unknowns.throttle                  = 0.8    *  ones_row(1)
            segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)    
            mission.append_segment(segment) 
           
            ## ------------------------------------------------------------------
            ##   First Descent Segment: Constant Acceleration, Constant Rate
            ## ------------------------------------------------------------------
        
            #segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            #segment.tag = "Descent_1"+ "_F_" + str(flight_no) + "_D" + str (day) 
            #segment.analyses.extend( analyses.base)  
            #segment.altitude_start                           = 2500.0 * Units.ft
            #segment.altitude_end                             = 100. * Units.ft   
            #segment.climb_rate                               = -300.  * Units['ft/min']
            #segment.air_speed_start                          = 175.  * Units['mph']
            #segment.air_speed_end                            = 1.1*Vstall    
            #segment.state.unknowns.throttle                  = 0.8   *  ones_row(1)  
            #segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)    
            #mission.append_segment(segment) 
          
            #if reserve_segment: 
                ## ------------------------------------------------------------------
                ##   Reserve Segment: Constant Acceleration, Constant Rate
                ## ------------------------------------------------------------------
            
                #segment     = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                #segment.tag = "Reserve_Climb" + "_F_" + str(flight_no) + "_D" + str (day) 
                #segment.analyses.extend( analyses.base ) 
                #segment.altitude_start           = 100.0 * Units.ft  
                #segment.altitude_end             = 1000. * Units.ft
                #segment.climb_rate               = 500.  * Units['ft/min']
                #segment.air_speed_start          = 1.1   * Vstall
                #segment.air_speed_end            = 150.  * Units['mph']  
                #segment.state.unknowns.throttle   = 0.9   *  ones_row(1)   
                #segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment) 
                #mission.append_segment(segment) 
              
            
                ## ------------------------------------------------------------------
                ##   Reserve Cruise Segment: Constant Acceleration, Constant Altitude
                ## ------------------------------------------------------------------
            
                #segment                                    = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
                #segment.tag                                = "Reserve_Cruise" + "_F_" + str(flight_no) + "_D" + str (day) 
                #segment.analyses.extend( analyses.base ) 
                #segment.altitude                           = 1000.0 * Units.ft
                #segment.air_speed                          = 150.   * Units['mph']
                #segment.distance                           = cruise_distance*0.1  + 6.6*Units.nmi
                #segment.state.unknowns.throttle            = 0.9    *  ones_row(1)
                #segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)    
                #mission.append_segment(segment)   
              
            
                ## ------------------------------------------------------------------
                ##   Reserve Descent Segment: Constant Acceleration, Constant Rate
                ## ------------------------------------------------------------------ 
                #segment                                  = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                #segment.tag                              = "Reserve_Descent" + "_F_" + str(flight_no) + "_D" + str (day) 
                #segment.analyses.extend( analyses.base )  
                #segment.altitude_start                   = 1000.0 * Units.ft
                #segment.altitude_end                     = 100. * Units.ft  
                #segment.climb_rate                       = -300.  * Units['ft/min']
                #segment.air_speed_start                  = 150.  * Units['mph']
                #segment.air_speed_end                    = 1.1*Vstall    
                #segment.state.unknowns.throttle          = 0.80    *  ones_row(1)
                #segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment,initial_prop_power_coefficient = 0.016)   
                #mission.append_segment(segment)        
             
            
            ## ------------------------------------------------------------------
            ##   First Cruise Segment: Transition
            ## ------------------------------------------------------------------ 
            #segment                                          = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
            #segment.tag                                      = "Approach_Transition" + "_F_" + str(flight_no) + "_D" + str (day) 
            #segment.analyses.extend( analyses.base ) 
            #segment.altitude_start                           = 100.0 * Units.ft  
            #segment.altitude_end                             = 40.0 * Units.ft  
            #segment.air_speed                                = 1.1*Vstall 
            #segment.climb_angle                              = 1 * Units.degrees
            #segment.acceleration                             = -0.3 * Units['m/s/s']    
            #segment.pitch_initial                            = 4. * Units.degrees     
            #segment.pitch_final                              = 5. * Units.degrees    
            #segment.state.unknowns.throttle                  = 0.70   *  ones_row(1) 
            #segment.process.iterate.unknowns.mission         = SUAVE.Methods.skip
            #segment.process.iterate.conditions.stability     = SUAVE.Methods.skip
            #segment.process.finalize.post_process.stability  = SUAVE.Methods.skip          
            #segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment,\
                                                                                            #initial_throttle_lift = 0.7) 
            #mission.append_segment(segment)  
        
            ## ------------------------------------------------------------------
            ##   Fifth Cuise Segment: Transition
            ## ------------------------------------------------------------------ 
            #segment                                         = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
            #segment.tag                                     = "Descent_Transition" + "_F_" + str(flight_no) + "_D" + str (day) 
            #segment.analyses.extend( analyses.desceleration ) 
            #segment.altitude                                = 40.  * Units.ft      
            #segment.air_speed_start                         = 103.5  * Units['mph']
            #segment.air_speed_end                           = 300. * Units['ft/min'] 
            #segment.acceleration                            = -0.75              
            #segment.pitch_initial                           =  5.  * Units.degrees  
            #segment.pitch_final                             = 10. * Units.degrees  
            #segment.state.unknowns.throttle                 = 0.8 *  ones_row(1)  
            #segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
            #segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
            #segment.process.finalize.post_process.stability = SUAVE.Methods.skip   
            #segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment,\
                                                                                            #initial_throttle_lift = 0.7) 
            #mission.append_segment(segment)       
         
            ## ------------------------------------------------------------------
            ##   Third Descent Segment: Constant Speed, Constant Rate
            ## ------------------------------------------------------------------ 
            #segment                                         = Segments.Hover.Descent(base_segment)
            #segment.tag                                     = "Vertical_Descent" + "_F_" + str(flight_no) + "_D" + str (day) 
            #segment.analyses.extend( analyses.base)     
            #segment.altitude_start                          = 40.0  * Units.ft  
            #segment.altitude_end                            = 0.   * Units.ft  
            #segment.descent_rate                            = 300. * Units['ft/min']   
            #segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
            #segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
            #segment.process.finalize.post_process.stability = SUAVE.Methods.skip 
            #segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment,\
                                                                                    #initial_lift_rotor_power_coefficient = 0.02,
                                                                                    #initial_throttle_lift = 0.7)
            #mission.append_segment(segment)
             
            #if recharge_battery:
                ## ------------------------------------------------------------------
                ##  Charge Segment: 
                ## ------------------------------------------------------------------  
                ## Charge Model 
                #segment                                                  = Segments.Ground.Battery_Charge_Discharge(base_segment)     
                #segment.tag                                              = 'Charge Day ' + "_F_" + str(flight_no) + "_D" + str (day)  
                #segment.analyses.extend(analyses.base)           
                #segment.battery_discharge                                = False   
                #if flight_no  == flights_per_day:  
                    #segment.increment_battery_cycle_day=True         
                #segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment)    
                #mission.append_segment(segment)       
            
  
    return mission 


# ------------------------------------------------------------------
#   Noise (Approach/Departure) Mission Setup
# ------------------------------------------------------------------
def approach_departure_mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range,
                                     reserve_segment,control_points,recharge_battery): 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # airport
    airport           = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976() 
    mission.airport    = airport      

    atmosphere         = SUAVE.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data          = atmosphere.compute_values(altitude = airport.altitude,temperature_deviation= 1.)     

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment           
    base_segment                                             = Segments.Segment() 
    base_segment.battery_discharge                           = True  
    base_segment.state.numerics.number_control_points        = control_points
    ones_row                                                 = base_segment.state.ones_row
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  
    base_segment.process.finalize.post_process.stability  = SUAVE.Methods.skip


    # VSTALL Calculation
    m      = vehicle.mass_properties.max_takeoff
    g      = 9.81
    S      = vehicle.reference_area
    atmo   = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    rho    = atmo.compute_values(1000.*Units.feet,0.).density
    CLmax  = 1.2 
    Vstall = float(np.sqrt(2.*m*g/(rho*S*CLmax))) 
     
    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Acceleration, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Descent_1" 
    segment.analyses.extend( analyses.descent)  
    segment.altitude_start                           = 500.0 * Units.ft
    segment.altitude_end                             = 100. * Units.ft   
    segment.climb_rate                               = -300.  * Units['ft/min']
    segment.air_speed_start                          = 175.  * Units['mph'] 
    segment.battery_energy                           = vehicle.networks.lift_cruise.battery.max_energy   
    segment.air_speed_end                            = 1.1*Vstall    
    segment.state.unknowns.throttle                  = 0.8   *  ones_row(1)  
    segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment,initial_prop_power_coefficient = 0.016)    
    mission.append_segment(segment)  
            
    # ------------------------------------------------------------------
    #   First Cruise Segment: Transition
    # ------------------------------------------------------------------ 
    segment                                          = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
    segment.tag                                      = "Approach_Transition"  
    segment.analyses.extend( analyses.base ) 
    segment.altitude_start                           = 100.0 * Units.ft  
    segment.altitude_end                             = 40.0 * Units.ft  
    segment.air_speed                                = 1.1*Vstall 
    segment.climb_angle                              = 1 * Units.degrees
    segment.acceleration                             = -0.3 * Units['m/s/s']    
    segment.pitch_initial                            = 4. * Units.degrees     
    segment.pitch_final                              = 1. * Units.degrees    
    segment.state.unknowns.throttle                  = 0.70   *  ones_row(1) 
    segment.process.iterate.unknowns.mission         = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability     = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability  = SUAVE.Methods.skip          
    segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment) 
    mission.append_segment(segment)  

    # ------------------------------------------------------------------
    #   Fifth Cuise Segment: Transition
    # ------------------------------------------------------------------ 
    segment                                         = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
    segment.tag                                     = "Descent_Transition"  
    segment.analyses.extend( analyses.desceleration ) 
    segment.altitude                                = 40.  * Units.ft      
    segment.air_speed_start                         = 103.5  * Units['mph']
    segment.air_speed_end                           = 300. * Units['ft/min'] 
    segment.acceleration                            = -0.75              
    segment.pitch_initial                           =  5.  * Units.degrees  
    segment.pitch_final                             = 20. * Units.degrees  
    segment.state.unknowns.throttle                 = 0.9 *  ones_row(1)  
    segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip   
    segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment,\
                                                                                    initial_lift_rotor_power_coefficient = 0.02,
                                                                                    initial_throttle_lift = 0.9) 
    mission.append_segment(segment)       
 
    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                                         = Segments.Hover.Descent(base_segment)
    segment.tag                                     = "Vertical_Descent" 
    segment.analyses.extend( analyses.base )     
    segment.altitude_start                          = 40.0  * Units.ft  
    segment.altitude_end                            = 0.   * Units.ft  
    segment.descent_rate                            = 300. * Units['ft/min']   
    segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip 
    segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment,\
                                                                                    initial_lift_rotor_power_coefficient = 0.02,
                                                                                    initial_throttle_lift = 0.7)
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment     = Segments.Hover.Climb(base_segment)
    segment.tag = "Vertical_Climb"  
    segment.analyses.extend( analyses.base )  
    segment.altitude_start                          = 0.0  * Units.ft  
    segment.altitude_end                            = 40.  * Units.ft  
    segment.climb_rate                              = 500. * Units['ft/min']   
    segment.battery_pack_temperature                   = atmo_data.temperature[0,0]             
    segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip
    segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment,\
                                                                            initial_lift_rotor_power_coefficient = 0.02,
                                                                            initial_throttle_lift = 0.7)
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   First Cruise Segment: Transition
    # ------------------------------------------------------------------
 
    segment                                    = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
    segment.tag                                = "Vertical_Transition"   
    segment.analyses.extend( analyses.transition ) 
    segment.altitude                           = 40.  * Units.ft              
    segment.air_speed_start                    = 500. * Units['ft/min']
    segment.air_speed_end                      = 0.8 * Vstall
    segment.acceleration                       = 1.5  
    segment.pitch_initial                      = 0.0 * Units.degrees
    segment.pitch_final                        = 7. * Units.degrees  
    segment.state.unknowns.throttle            = 0.70    *  ones_row(1) 
    segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip
    segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment,
                                                         initial_prop_power_coefficient = 0.1, 
                                                         initial_lift_rotor_power_coefficient = 0.016,
                                                         initial_throttle_lift = 0.9) 
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   First Cruise Segment: Transition
    # ------------------------------------------------------------------ 
    segment                                      = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
    segment.tag                                  = "Climb_Transition" 
    segment.analyses.extend( analyses.base ) 
    segment.altitude_start                       = 40.0 * Units.ft  
    segment.altitude_end                         = 100.0 * Units.ft  
    segment.air_speed                            = 0.8   * Vstall
    segment.climb_angle                          = 2     * Units.degrees
    segment.acceleration                         = 0.25   * Units['m/s/s']    
    segment.pitch_initial                        = 7.    * Units.degrees  
    segment.pitch_final                          = 5.    * Units.degrees   
    segment.state.unknowns.throttle              = 0.70   *  ones_row(1)
    segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip 
    segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment,
                                                         initial_prop_power_coefficient = 0.1,
                                                         initial_lift_rotor_power_coefficient = 0.016,
                                                         initial_throttle_lift = 0.9) 
    mission.append_segment(segment) 
  
    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                                      = "Climb_1"   
    segment.analyses.extend( analyses.base ) 
    segment.altitude_start                           = 100.0 * Units.ft  
    segment.altitude_end                             = 500. * Units.ft  
    segment.climb_rate                               = 600.  * Units['ft/min']
    segment.air_speed_start                          = 95.   * Units['mph']
    segment.air_speed_end                            = Vstall*1.2  
    segment.state.unknowns.throttle                  = 0.9   *  ones_row(1)  
    segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)   
    mission.append_segment(segment)  

    return mission  

# ------------------------------------------------------------------
#  Hover Noise Mission Setup
# ------------------------------------------------------------------   
def hover_mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range,
                        reserve_segment,control_points,recharge_battery):
 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # airport
    airport           = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976() 
    mission.airport    = airport      

    atmosphere         = SUAVE.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data          = atmosphere.compute_values(altitude = airport.altitude,temperature_deviation= 1.)     

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment           
    base_segment                                             = Segments.Segment() 
    base_segment.battery_discharge                           = True  
    base_segment.state.numerics.number_control_points        = 3 
    base_segment.process.initialize.initialize_battery       = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  
    base_segment.process.finalize.post_process.stability  = SUAVE.Methods.skip
 
    segment                                            = Segments.Hover.Climb(base_segment)
    segment.tag                                        = "Hover" # very small climb so that broadband noise does not return 0's  
    segment.analyses.extend(analyses.base) 
    segment.altitude_start                             = 500.0  * Units.ft  
    segment.altitude_end                               = 500.1  * Units.ft 
    segment.climb_rate                                 = 100. * Units['ft/min']  
    segment.battery_energy                             = vehicle.networks.lift_cruise.battery.max_energy   
    segment.battery_pack_temperature                   = atmo_data.temperature[0,0]  
    segment.process.iterate.unknowns.mission           = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability       = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability    = SUAVE.Methods.skip
    segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment,\
                                                                            initial_lift_rotor_power_coefficient = 0.02,
                                                                            initial_throttle_lift = 0.7)
    mission.append_segment(segment)
  
    
    return mission


# ----------------------------------------------------------------------
#   Missions Setup
# ----------------------------------------------------------------------
def missions_setup(base_mission):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission


    # done!
    return missions  