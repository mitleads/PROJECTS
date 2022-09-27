# Cora.py
# 
# Created:  April 2018, SUAVE Team
# Modified: Sep 2018, Emilio Botero, Wally Maier

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Components.Energy.Networks.Lift_Cruise import Lift_Cruise 
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_mass
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Aerodynamics.Fidelity_Zero.Lift import compute_max_lift_coeff
from SUAVE.Methods.Noise.Fidelity_Zero.Propeller import propeller_noise_frequency
from SUAVE.Methods.Noise.Correlations.propeller_noise_sae import propeller_noise_sae
from SUAVE.Methods.Weights.Buildups.eVTOL.empty import empty

import numpy as np
import pylab as plt
from copy import deepcopy

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():

    # build the vehicle, configs, and analyses
    configs, analyses = full_setup()

    # configs.finalize()
    analyses.finalize()    

    # weight analysis
    weights = analyses.weights
    # breakdown = weights.evaluate()          

    # mission analysis
    mission = analyses.mission
    results = mission.evaluate()

    # Do noise calcs
    #results = noise_analysis(results,configs)

    # save results 
    #save_results(results,configs)    

    # plot results
    plot_mission(results)

    return

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------
def full_setup():

    # vehicle data
    vehicle  = vehicle_setup()

    # vehicle analyses
    analyses = base_analysis(vehicle)

    # mission analyses
    mission  = mission_setup(analyses,vehicle)

    # analyses = SUAVE.Analyses.Analysis.Container()
    # analyses.configs  = configs_analyses
    analyses.mission = mission

    return vehicle, analyses

# ----------------------------------------------------------------------
#   Noise Results
# ----------------------------------------------------------------------
def noise_analysis(results,vehicle):

    for i in range(len(results.segments)):  

        noise_data_forward  = results.segments[i].conditions.propulsion.acoustic_outputs.Forward_Prop
        noise_data_lift     = results.segments[i].conditions.propulsion.acoustic_outputs.Lift_Prop

        # Go through the data
        noise_data_forward.diameter    = 2 * vehicle.propulsors.propulsor.propeller.tip_radius
        noise_data_forward.n_blades    = vehicle.propulsors.propulsor.propeller.number_of_blades
        noise_data_forward.n_engines   = vehicle.propulsors.propulsor.number_of_engines_forward
        noise_data_forward.sound_speed = results.segments[i].conditions.freestream.speed_of_sound
        noise_data_forward.speed       = results.segments[i].conditions.freestream.velocity
        noise_data_forward.angle       = np.array([90. * Units.degrees])
        noise_data_forward.distance    = np.array([10.])

        noise_data_lift.diameter    = 2 * vehicle.propulsors.propulsor.rotor.tip_radius
        noise_data_lift.n_blades    = vehicle.propulsors.propulsor.rotor.number_of_blades
        noise_data_lift.n_engines   = vehicle.propulsors.propulsor.number_of_engines_forward
        noise_data_lift.sound_speed = results.segments[i].conditions.freestream.speed_of_sound
        noise_data_lift.speed       = results.segments[i].conditions.freestream.velocity
        noise_data_lift.angle       = np.array([90. * Units.degrees])
        noise_data_lift.distance    = np.array([10.])

        #(PNL_dBA_forward, EPNdB_takeoff_forward, EPNdB_landing_forward) = propeller_noise_sae(noise_data_forward)
        (PNL_dBA_lift, EPNdB_takeoff_lift, EPNdB_landing_lift)          =  propeller_noise_sae(noise_data_lift)

        #noise_data_forward.PNL_dBA       = PNL_dBA_forward
        #noise_data_forward.EPNdB_takeoff = EPNdB_takeoff_forward
        #noise_data_forward.EPNdB_landing = EPNdB_landing_forward 

        noise_data_lift.PNL_dBA       = PNL_dBA_lift
        noise_data_lift.EPNdB_takeoff = EPNdB_takeoff_lift
        noise_data_lift.EPNdB_landing = EPNdB_landing_lift

    return results

# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def vehicle_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = SUAVE.Vehicle(SUAVE.Input_Output.SUAVE.load('cora_vehicle.res'))

    vehicle.tag = 'Cora'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.takeoff           = 2450. * Units.lb
    vehicle.mass_properties.operating_empty   = 2250. * Units.lb
    vehicle.mass_properties.max_takeoff       = 2450. * Units.lb
    vehicle.mass_properties.center_of_gravity = [ 2.0144,   0.  ,  0.]


    # This needs updating
    # basic parameters
    vehicle.reference_area                    = vehicle.wings.main_wing.areas.reference   
    vehicle.envelope.ultimate_load            = 5.7
    vehicle.envelope.limit_load               = 3.8

    vehicle.wings['main_wing'].motor_spanwise_locations = np.multiply(
        2./36.25,
        [-5.435, -5.435, -9.891, -9.891, -14.157, -14.157,
         5.435, 5.435, 9.891, 9.891, 14.157, 14.157])

    vehicle.wings['main_wing'].winglet_fraction = 0.0

    vehicle.wings['main_wing'].thickness_to_chord = 0.18
    vehicle.wings['main_wing'].chords.mean_aerodynamic = 0.9644599977664836

    #------------------------------------------------------------------
    # Propulsor
    #------------------------------------------------------------------

    net = Lift_Cruise()
    net.number_of_engines_lift    = 12
    net.number_of_engines_forward = 1
    net.thrust_angle_lift         = 88. * Units.degrees
    net.thrust_angle_forward      = 0. 
    net.areas                     = Data()
    net.areas.wetted              = .05
    net.nacelle_diameter          = 0.0001
    net.engine_length             = 0.001
    net.number_of_engines         = 1.
    net.voltage                   = 500. #400

    esc_lift                      = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc_lift.efficiency           = 0.95
    net.esc_lift                  = esc_lift

    esc_thrust                    = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc_thrust.efficiency         = 0.95
    net.esc_forward               = esc_thrust

    payload                       = SUAVE.Components.Energy.Peripherals.Avionics()
    payload.power_draw            = 0.
    payload.mass_properties.mass  = 200. * Units.kg
    net.payload                   = payload

    avionics                      = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw           = 200. * Units.watts
    net.avionics                  = avionics

    bat                           = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.specific_energy           = 300. * Units.Wh/Units.kg
    bat.resistance                = 0.005
    bat.max_voltage               = net.voltage 
    bat.mass_properties.mass      = 300. * Units.kg
    initialize_from_mass(bat, bat.mass_properties.mass)
    net.battery                   = bat

        # Component 5 the thrust propeller
    # Design the thrust propeller
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.tip_radius          = vehicle.prop_forward.tip_radius
    prop.number_of_blades    = vehicle.prop_forward.number_of_blades
    prop.twist_distribution  = vehicle.prop_forward.twist_distribution
    prop.chord_distribution  = vehicle.prop_forward.chord_distribution
    prop.radius_distribution = vehicle.prop_forward.radius_distribution
    prop.origin              = vehicle.prop_forward.origin
    prop.tag                 = 'Forward_Prop'

    net.propeller_forward    = prop

    # Lift propellers  
    prop_lift = SUAVE.Components.Energy.Converters.Propeller()
    prop_lift.tip_radius          = vehicle.prop_lift.tip_radius
    prop_lift.number_of_blades    = vehicle.prop_lift.number_of_blades
    prop_lift.twist_distribution  = vehicle.prop_lift.twist_distribution
    prop_lift.chord_distribution  = vehicle.prop_lift.chord_distribution
    prop_lift.radius_distribution = vehicle.prop_lift.radius_distribution
    prop_lift.origin              = vehicle.prop_lift.origin    
    prop_lift.tag                 = 'Lift_Prop'

    vehicle_weight                = vehicle.mass_properties.takeoff*9.81    
    rho                           = 1.225
    A                             = np.pi*(prop_lift.tip_radius**2)
    prop_lift.induced_hover_velocity = np.sqrt(vehicle_weight/(2*rho*A*net.number_of_engines_lift))

    net.propeller_lift           = prop_lift

    # Motors
    # Thrust motor
    motor_thrust = SUAVE.Components.Energy.Converters.Motor()
    motor_thrust.mass_properties.mass = 25. * Units.kg
    motor_thrust = size_from_mass(motor_thrust)

    etam = 0.95
    v    = 300  #bat.max_voltage *3/4
    omeg = 2500. * Units.rpm
    kv   = 8.5 * Units.rpm
    io   = 2.0

    res  = ((v-omeg/kv)*(1.-etam*v*kv/omeg))/io

    motor_thrust.speed_constant  = kv
    motor_thrust.resistance      = res
    motor_thrust.no_load_current = io  

    motor_thrust.gear_ratio = 1.0
    motor_thrust.propeller_radius = prop.tip_radius
    net.motor_forward = motor_thrust

    #Lift Motor
    motor_lift = SUAVE.Components.Energy.Converters.Motor()
    motor_lift.mass_properties.mass = 3. * Units.kg
    #motor_lift = size_from_mass(motor_lift, motor_lift.mass_properties.mass)

    etam = 0.95
    v    = 400 #bat.max_voltage 
    omeg = 3000. * Units.rpm
    kv   = 8.0 * Units.rpm  
    io   = 4.0

    res  = ((v-omeg/kv)*(1.-etam*v*kv/omeg))/io


    motor_lift.speed_constant  = kv
    motor_lift.resistance      = res
    motor_lift.no_load_current = io

    motor_lift.gear_ratio         = 1.0
    motor_lift.gearbox_efficiency = 1.0
    motor_lift.propeller_radius   = prop_lift.tip_radius
    net.motor_lift                = motor_lift

    vehicle.append_component(net)

    # Add extra drag sources from motors, props, and landing gear. All of these hand measured

    motor_height     = .25 * Units.feet
    motor_width      =  1.6 * Units.feet

    propeller_width  = 1. * Units.inches
    propeller_height = propeller_width *.12

    main_gear_width  = 1.5 * Units.inches
    main_gear_length = 2.5 * Units.feet

    nose_gear_width  = 2. * Units.inches
    nose_gear_length = 2. * Units.feet

    nose_tire_height = (0.7 + 0.4) * Units.feet
    nose_tire_width  = 0.4 * Units.feet

    main_tire_height = (0.75 + 0.5) * Units.feet
    main_tire_width  = 4. * Units.inches

    total_excrescence_area_spin = 12.*motor_height*motor_width + \
        2.*main_gear_length*main_gear_width + nose_gear_width*nose_gear_length + \
        2*main_tire_height*main_tire_width + nose_tire_height*nose_tire_width

    total_excrescence_area_no_spin = total_excrescence_area_spin + 12*propeller_height*propeller_width 

    vehicle.excrescence_area_no_spin = total_excrescence_area_no_spin 
    vehicle.excrescence_area_spin = total_excrescence_area_spin     

    vehicle.weight_breakdown = empty(vehicle)

    return vehicle

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_eVTOL()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.4*vehicle.excrescence_area_spin / vehicle.reference_area
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    return analyses    


def mission_setup(analyses,vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    ones_row     = base_segment.state.ones_row
    base_segment.state.numerics.number_control_points        = 10
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip
    base_segment.process.iterate.unknowns.network            = vehicle.propulsors.propulsor.unpack_unknowns_transition
    base_segment.process.iterate.residuals.network           = vehicle.propulsors.propulsor.residuals_transition
    base_segment.state.unknowns.battery_voltage_under_load   = 400. * ones_row(1) #vehicle.propulsors.propulsor.battery.max_voltage * ones_row(1)  
    base_segment.state.residuals.network                     = 0. * ones_row(2)    


    # VSTALL Calculation
    m     = vehicle.mass_properties.max_takeoff
    g     = 9.81
    S     = vehicle.reference_area
    atmo  = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    rho   = atmo.compute_values(1000.*Units.feet,0.).density
    CLmax = 1.2



    Vstall = float(np.sqrt(2.*m*g/(rho*S*CLmax)))

    # ------------------------------------------------------------------
    #   First Taxi Segment: Constant Speed
    # ------------------------------------------------------------------

    segment = Segments.Hover.Climb(base_segment)
    segment.tag = "Ground_Taxi"

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Hover.Climb(base_segment)
    segment.tag = "climb_1"

    segment.analyses.extend( analyses )

    segment.altitude_start  = 0.0  * Units.ft
    segment.altitude_end    = 40.  * Units.ft
    segment.climb_rate      = 500. * Units['ft/min']

    segment.battery_energy  = vehicle.propulsors.propulsor.battery.max_energy * 0.95  

    segment.state.unknowns.propeller_power_coefficient_lift = 0.04 * ones_row(1)
    segment.state.unknowns.throttle_lift                    = 0.85 * ones_row(1)
    segment.state.unknowns.__delitem__('throttle')

    segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_forward
    segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_forward       
    segment.process.iterate.unknowns.mission  = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability      = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability   = SUAVE.Methods.skip

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Cruise Segment: Transition
    # ------------------------------------------------------------------

    segment = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
    segment.tag = "transition_1"

    segment.analyses.extend( analyses )

    segment.altitude        = 40.  * Units.ft
    segment.air_speed_start = 0.   * Units['ft/min']
    segment.air_speed_end   = 1.2 * Vstall
    segment.acceleration    = 9.81/5
    segment.pitch_initial   = 0.0
    segment.pitch_final     = 7.75 * Units.degrees

    segment.state.unknowns.propeller_power_coefficient_lift = 0.07 * ones_row(1)
    segment.state.unknowns.throttle_lift                    = 0.70 * ones_row(1)
    segment.state.unknowns.propeller_power_coefficient      = 0.02 * ones_row(1)
    segment.state.unknowns.throttle                         = .50  * ones_row(1)   
    segment.state.residuals.network                         = 0.   * ones_row(3)    

    segment.process.iterate.unknowns.network                = vehicle.propulsors.propulsor.unpack_unknowns_transition
    segment.process.iterate.residuals.network               = vehicle.propulsors.propulsor.residuals_transition    
    segment.process.iterate.unknowns.mission                = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability         = SUAVE.Methods.skip

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2"

    segment.analyses.extend( analyses )

    segment.air_speed       = np.sqrt((500 * Units['ft/min'])**2 + (1.2*Vstall)**2)
    segment.altitude_start  = 40.0 * Units.ft
    segment.altitude_end    = 300. * Units.ft
    segment.climb_rate      = 500. * Units['ft/min']

    segment.state.unknowns.propeller_power_coefficient         = 0.01 * ones_row(1)
    segment.state.unknowns.throttle                            = 0.70 * ones_row(1)
    segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_lift     

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude_Loiter(base_segment)
    segment.tag = "departure_terminal_procedures"

    segment.analyses.extend( analyses )

    segment.altitude  = 300.0 * Units.ft
    segment.time      = 60.   * Units.second
    segment.air_speed = 1.2*Vstall

    segment.state.unknowns.propeller_power_coefficient = 0.01 * ones_row(1)
    segment.state.unknowns.throttle                    = 0.50 * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_lift     


    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Climb Segment: Constant Acceleration, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "accelerated_climb"

    segment.analyses.extend( analyses )

    segment.altitude_start  = 300.0 * Units.ft
    segment.altitude_end    = 1000. * Units.ft
    segment.climb_rate      = 500.  * Units['ft/min']
    segment.air_speed_start = np.sqrt((500 * Units['ft/min'])**2 + (1.2*Vstall)**2)
    segment.air_speed_end   = 110.  * Units['mph']                                            

    segment.state.unknowns.propeller_power_coefficient = 0.01 * ones_row(1)
    segment.state.unknowns.throttle                    = 0.50 * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_lift  


    # add to misison
    mission.append_segment(segment)    

    # ------------------------------------------------------------------
    #   Third Cruise Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses )

    segment.altitude  = 1000.0 * Units.ft
    segment.air_speed = 110.   * Units['mph']
    segment.distance  = 60.    * Units.miles                       

    segment.state.unknowns.propeller_power_coefficient = 0.02 * ones_row(1)
    segment.state.unknowns.throttle                    = 0.40 * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_lift    


    # add to misison
    mission.append_segment(segment)     

    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Acceleration, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "decelerating_descent"

    segment.analyses.extend( analyses )  
    segment.altitude_start  = 1000.0 * Units.ft
    segment.altitude_end    = 300. * Units.ft
    segment.climb_rate      = -500.  * Units['ft/min']
    segment.air_speed_start = 110.  * Units['mph']
    segment.air_speed_end   = 1.2*Vstall

    segment.state.unknowns.propeller_power_coefficient = 0.03 * ones_row(1)
    segment.state.unknowns.throttle                    = 0.5 * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_lift     

    # add to misison
    mission.append_segment(segment)        

    # ------------------------------------------------------------------
    #   Fourth Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude_Loiter(base_segment)
    segment.tag = "arrival_terminal_procedures"

    segment.analyses.extend( analyses )

    segment.altitude        = 300.   * Units.ft
    segment.air_speed       = 1.2*Vstall
    segment.time            = 60 * Units.seconds

    segment.state.unknowns.propeller_power_coefficient = 0.01 * ones_row(1)
    segment.state.unknowns.throttle                    = 0.50 * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_lift   

    # add to misison
    mission.append_segment(segment)    

    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_2"

    segment.analyses.extend( analyses )

    segment.altitude_start  = 300.0 * Units.ft
    segment.altitude_end    = 40. * Units.ft
    segment.climb_rate      = -400.  * Units['ft/min']  # Uber has 500->300
    segment.air_speed_start = np.sqrt((400 * Units['ft/min'])**2 + (1.2*Vstall)**2)
    segment.air_speed_end   = 1.2*Vstall                           

    segment.state.unknowns.propeller_power_coefficient = 0.01 * ones_row(1)
    segment.state.unknowns.throttle                    = 0.50 * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_lift 


    # add to misison
    mission.append_segment(segment)       

    # ------------------------------------------------------------------
    #   Fifth Cuise Segment: Transition
    # ------------------------------------------------------------------ 
    segment = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
    segment.tag = "transition_2"

    segment.analyses.extend( analyses )

    segment.altitude        = 40. * Units.ft
    segment.air_speed_start = 1.2 * Vstall      
    segment.air_speed_end   = 0 
    segment.acceleration    = -9.81/20
    segment.pitch_initial   = 5. * Units.degrees   
    segment.pitch_final     = 10. * Units.degrees      
    
    segment.state.unknowns.propeller_power_coefficient_lift = 0.075 * ones_row(1)  
    segment.state.unknowns.throttle_lift                    = 0.75   * ones_row(1)    
    segment.state.unknowns.propeller_power_coefficient      = 0.03  * ones_row(1)  
    segment.state.unknowns.throttle                         = 0.5   * ones_row(1)  
    segment.state.residuals.network                         = 0.    * ones_row(3)    
    
    segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_transition
    segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_transition    
    segment.process.iterate.unknowns.mission  = SUAVE.Methods.skip 
    # add to misison
    mission.append_segment(segment)

      
    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Hover.Descent(base_segment)
    segment.tag = "descent_1"

    segment.analyses.extend( analyses )

    segment.altitude_start  = 40.0  * Units.ft
    segment.altitude_end    = 0.   * Units.ft
    segment.descent_rate    = 300. * Units['ft/min']
    segment.battery_energy  = vehicle.propulsors.propulsor.battery.max_energy

    segment.state.unknowns.propeller_power_coefficient_lift = 0.05* ones_row(1)
    segment.state.unknowns.throttle_lift                   = 0.9 * ones_row(1)

    segment.state.unknowns.__delitem__('throttle')
    segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_forward
    segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_forward    
    segment.process.iterate.unknowns.mission  = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability      = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability   = SUAVE.Methods.skip

    # add to misison
    mission.append_segment(segment)       

    ## ------------------------------------------------------------------
    ##
    ##   Reserve Mission
    ##
    ## ------------------------------------------------------------------

    ## ------------------------------------------------------------------
    ##   First Climb Segment: Constant Speed, Constant Rate
    ## ------------------------------------------------------------------

    #segment = Segments.Hover.Climb(base_segment)
    #segment.tag = "reserve_climb_1"

    #segment.analyses.extend( analyses )

    #segment.altitude_start  = 0.0  * Units.ft
    #segment.altitude_end    = 40.  * Units.ft
    #segment.climb_rate      = 500. * Units['ft/min']

    #segment.battery_energy  = vehicle.propulsors.propulsor.battery.max_energy * 0.95  

    #segment.state.unknowns.propeller_power_coefficient_lift = 0.04 * ones_row(1)
    #segment.state.unknowns.throttle_lift                    = 0.85 * ones_row(1)
    #segment.state.unknowns.__delitem__('throttle')

    #segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_forward
    #segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_forward       
    #segment.process.iterate.unknowns.mission  = SUAVE.Methods.skip
    #segment.process.iterate.conditions.stability      = SUAVE.Methods.skip
    #segment.process.finalize.post_process.stability   = SUAVE.Methods.skip

    ## add to misison
    #mission.append_segment(segment)

    ## ------------------------------------------------------------------
    ##   First Cruise Segment: Transition
    ## ------------------------------------------------------------------

    #segment = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
    #segment.tag = "reserve_transition_1"

    #segment.analyses.extend( analyses )

    #segment.altitude        = 40.  * Units.ft
    #segment.air_speed_start = 0.   * Units['ft/min']
    #segment.air_speed_end   = 1.2 * Vstall
    #segment.acceleration    = 9.81/5
    #segment.pitch_initial   = 0.0
    #segment.pitch_final     = 7.75 * Units.degrees

    #segment.state.unknowns.propeller_power_coefficient_lift = 0.07 * ones_row(1)
    #segment.state.unknowns.throttle_lift                    = 0.70 * ones_row(1)
    #segment.state.unknowns.propeller_power_coefficient      = 0.02 * ones_row(1)
    #segment.state.unknowns.throttle                         = .50  * ones_row(1)   
    #segment.state.residuals.network                         = 0.   * ones_row(3)    

    #segment.process.iterate.unknowns.network                = vehicle.propulsors.propulsor.unpack_unknowns_transition
    #segment.process.iterate.residuals.network               = vehicle.propulsors.propulsor.residuals_transition    
    #segment.process.iterate.unknowns.mission                = SUAVE.Methods.skip
    #segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    #segment.process.finalize.post_process.stability         = SUAVE.Methods.skip

    ## add to misison
    #mission.append_segment(segment)

    ## ------------------------------------------------------------------
    ##   Second Climb Segment: Constant Speed, Constant Rate
    ## ------------------------------------------------------------------

    #segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    #segment.tag = "reserve_climb_2"

    #segment.analyses.extend( analyses )

    #segment.air_speed       = np.sqrt((500 * Units['ft/min'])**2 + (1.2*Vstall)**2)
    #segment.altitude_start  = 40.0 * Units.ft
    #segment.altitude_end    = 300. * Units.ft
    #segment.climb_rate      = 500. * Units['ft/min']

    #segment.state.unknowns.propeller_power_coefficient         = 0.01 * ones_row(1)
    #segment.state.unknowns.throttle                            = 0.70 * ones_row(1)
    #segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_lift     

    ## add to misison
    #mission.append_segment(segment)

    ## ------------------------------------------------------------------
    ##   Second Cruise Segment: Constant Speed, Constant Altitude
    ## ------------------------------------------------------------------

    #segment = Segments.Cruise.Constant_Speed_Constant_Altitude_Loiter(base_segment)
    #segment.tag = "reserve_departure_terminal_procedures"

    #segment.analyses.extend( analyses )

    #segment.altitude  = 300.0 * Units.ft
    #segment.time      = 60.   * Units.second
    #segment.air_speed = 1.2*Vstall

    #segment.state.unknowns.propeller_power_coefficient = 0.01 * ones_row(1)
    #segment.state.unknowns.throttle                    = 0.50 * ones_row(1)

    #segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_lift     


    ## add to misison
    #mission.append_segment(segment)

    ## ------------------------------------------------------------------
    ##   Third Climb Segment: Constant Acceleration, Constant Rate
    ## ------------------------------------------------------------------

    #segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    #segment.tag = "reserve_accelerated_climb"

    #segment.analyses.extend( analyses )

    #segment.altitude_start  = 300.0 * Units.ft
    #segment.altitude_end    = 1000. * Units.ft
    #segment.climb_rate      = 500.  * Units['ft/min']
    #segment.air_speed_start = np.sqrt((500 * Units['ft/min'])**2 + (1.2*Vstall)**2)
    #segment.air_speed_end   = 110.  * Units['mph']                                            

    #segment.state.unknowns.propeller_power_coefficient = 0.01 * ones_row(1)
    #segment.state.unknowns.throttle                    = 0.50 * ones_row(1)

    #segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_lift  


    ## add to misison
    #mission.append_segment(segment)    

    ## ------------------------------------------------------------------
    ##   Third Cruise Segment: Constant Acceleration, Constant Altitude
    ## ------------------------------------------------------------------

    #segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    #segment.tag = "reserve_cruise"

    #segment.analyses.extend( analyses )

    #segment.altitude  = 1000.0 * Units.ft
    #segment.air_speed = 110.   * Units['mph']
    #segment.distance  = 6.    * Units.miles                       

    #segment.state.unknowns.propeller_power_coefficient = 0.01 * ones_row(1)
    #segment.state.unknowns.throttle                    = 0.50 * ones_row(1)

    #segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_lift    


    ## add to misison
    #mission.append_segment(segment)     

    ## ------------------------------------------------------------------
    ##   First Descent Segment: Constant Acceleration, Constant Rate
    ## ------------------------------------------------------------------

    #segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    #segment.tag = "reserve_decelerating_descent"

    #segment.analyses.extend( analyses )  
    #segment.altitude_start  = 1000.0 * Units.ft
    #segment.altitude_end    = 300. * Units.ft
    #segment.climb_rate      = -500.  * Units['ft/min']
    #segment.air_speed_start = 110.  * Units['mph']
    #segment.air_speed_end   = 1.2*Vstall

    #segment.state.unknowns.propeller_power_coefficient = 0.05 * ones_row(1)
    #segment.state.unknowns.throttle                    = 0.75 * ones_row(1)

    #segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_lift     

    ## add to misison
    #mission.append_segment(segment)        

    ## ------------------------------------------------------------------
    ##   Fourth Cruise Segment: Constant Speed, Constant Altitude
    ## ------------------------------------------------------------------

    #segment = Segments.Cruise.Constant_Speed_Constant_Altitude_Loiter(base_segment)
    #segment.tag = "reserve_arrival_terminal_procedures"

    #segment.analyses.extend( analyses )

    #segment.altitude        = 300.   * Units.ft
    #segment.air_speed       = 1.2*Vstall
    #segment.time            = 60 * Units.seconds

    #segment.state.unknowns.propeller_power_coefficient = 0.01 * ones_row(1)
    #segment.state.unknowns.throttle                    = 0.50 * ones_row(1)

    #segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_lift   

    ## add to misison
    #mission.append_segment(segment)    

    ## ------------------------------------------------------------------
    ##   Second Descent Segment: Constant Speed, Constant Rate
    ## ------------------------------------------------------------------

    #segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    #segment.tag = "reserve_descent_2"

    #segment.analyses.extend( analyses )

    #segment.altitude_start  = 300.0 * Units.ft
    #segment.altitude_end    = 40. * Units.ft
    #segment.climb_rate      = -400.  * Units['ft/min']  # Uber has 500->300
    #segment.air_speed_start = np.sqrt((400 * Units['ft/min'])**2 + (1.2*Vstall)**2)
    #segment.air_speed_end   = 1.2*Vstall                           

    #segment.state.unknowns.propeller_power_coefficient = 0.01 * ones_row(1)
    #segment.state.unknowns.throttle                    = 0.50 * ones_row(1)

    #segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_lift 


    ## add to misison
    #mission.append_segment(segment)       

    ## ------------------------------------------------------------------
    ##   Fifth Cuise Segment: Transition
    ## ------------------------------------------------------------------ 
    #segment = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
    #segment.tag = "reserve_transition_2"

    #segment.analyses.extend( analyses )

    #segment.altitude        = 40. * Units.ft
    #segment.air_speed_start = 1.2 * Vstall      
    #segment.air_speed_end   = 0 
    #segment.acceleration    = -9.81/20
    #segment.pitch_initial   = 5. * Units.degrees   
    #segment.pitch_final     = 10. * Units.degrees      
    
    #segment.state.unknowns.propeller_power_coefficient_lift = 0.075 * ones_row(1)  
    #segment.state.unknowns.throttle_lift                    = 0.75   * ones_row(1)    
    #segment.state.unknowns.propeller_power_coefficient      = 0.03  * ones_row(1)  
    #segment.state.unknowns.throttle                         = 0.5   * ones_row(1)  
    #segment.state.residuals.network                         = 0.    * ones_row(3)    
    
    #segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_transition
    #segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_transition    
    #segment.process.iterate.unknowns.mission  = SUAVE.Methods.skip 
    ## add to misison
    #mission.append_segment(segment)

      
    ## ------------------------------------------------------------------
    ##   Third Descent Segment: Constant Speed, Constant Rate
    ## ------------------------------------------------------------------

    #segment = Segments.Hover.Descent(base_segment)
    #segment.tag = "reserve_descent_1"

    #segment.analyses.extend( analyses )

    #segment.altitude_start  = 40.0  * Units.ft
    #segment.altitude_end    = 0.   * Units.ft
    #segment.descent_rate    = 300. * Units['ft/min']
    #segment.battery_energy  = vehicle.propulsors.propulsor.battery.max_energy

    #segment.state.unknowns.propeller_power_coefficient_lift = 0.05* ones_row(1)
    #segment.state.unknowns.throttle_lift                   = 0.9 * ones_row(1)

    #segment.state.unknowns.__delitem__('throttle')
    #segment.process.iterate.unknowns.network  = vehicle.propulsors.propulsor.unpack_unknowns_no_forward
    #segment.process.iterate.residuals.network = vehicle.propulsors.propulsor.residuals_no_forward    
    #segment.process.iterate.unknowns.mission  = SUAVE.Methods.skip
    #segment.process.iterate.conditions.stability      = SUAVE.Methods.skip
    #segment.process.finalize.post_process.stability   = SUAVE.Methods.skip

    ## add to misison
    #mission.append_segment(segment)       
    
    ## ------------------------------------------------------------------
    ##   Mission definition complete    
    ## ------------------------------------------------------------------

    return mission

# ----------------------------------------------------------------------
#   Plot Results
# ----------------------------------------------------------------------
def plot_mission(results): 
    plot_battery_pack_conditions(results) 
    plot_lift_cruise_network(results) 
    return     


def save_results(results,config):

    prop_radius_ft_forward = config.propulsors.propulsor.propeller.tip_radius*3.28084 # conversion to ft 
    prop_radius_ft_lift    = config.propulsors.propulsor.rotor.tip_radius*3.28084 # conversion to ft 

    j = 0
    n = len(results.segments[0].conditions.frames.inertial.time[:,0])
    seg_count  = len(results.segments)
    result_mat = np.zeros((seg_count*n,37))
    for segment in results.segments.values():    
        for i in range(n):
            time           = segment.conditions.frames.inertial.time[i,0] / Units.min

            CLift          = segment.conditions.aerodynamics.lift_coefficient[i,0]
            CDrag          = segment.conditions.aerodynamics.drag_coefficient[i,0]
            AoA            = segment.conditions.aerodynamics.angle_of_attack[i,0] / Units.deg
            l_d            = CLift/CDrag

            eta_l          = segment.conditions.propulsion.throttle_lift[i,0]
            eta_f          = segment.conditions.propulsion.throttle[i,0]
            energy         = segment.conditions.propulsion.battery_energy[i,0] 
            volts          = segment.conditions.propulsion.voltage_under_load[i,0] 
            volts_oc       = segment.conditions.propulsion.voltage_open_circuit[i,0]  

            current_lift    = segment.conditions.propulsion.current_lift[i,0]  
            current_forward = segment.conditions.propulsion.current_forward[i,0]  
            battery_amp_hr = (energy*0.000277778)/volts
            C_rating       = current_lift/battery_amp_hr 
            altitude       = segment.conditions.freestream.altitude[i,0]

            rpm_lift       = segment.conditions.propulsion.rpm_lift [i,0]
            rps_lift       = rpm_lift/60
            thrust_lift    = segment.conditions.frames.body.thrust_force_vector[i,2]
            torque_lift    = segment.conditions.propulsion.motor_torque_lift[i,0]
            effp_lift      = segment.conditions.propulsion.propeller_efficiency_lift[i,0]
            effm_lift      = segment.conditions.propulsion.motor_efficiency_lift[i,0]

            rpm_forward    = segment.conditions.propulsion.rpm_forward [i,0] 
            rps_forward    = rpm_forward/60
            thrust_forward = segment.conditions.frames.body.thrust_force_vector[i,0]
            torque_forward = segment.conditions.propulsion.motor_torque_forward[i,0]
            effp_forward   = segment.conditions.propulsion.propeller_efficiency_forward[i,0]
            effm_forward   = segment.conditions.propulsion.motor_efficiency_forward[i,0]

            spec_power_in_bat       = segment.conditions.propulsion.battery_specfic_power[i,0]                 # converting W to hp   
            power_in_bat_forward    = -segment.conditions.propulsion.battery_power_draw.forward_prop[i,0]*0.00134102 # converting W to hp      
            power_in_bat_lift       = -segment.conditions.propulsion.battery_power_draw.lift_prop[i,0]*0.00134102    # converting W to hp      
            power_in_shaft_lift     = torque_lift/(2*np.pi*rps_lift)*0.00134102                                # converting W to hp  
            power_in_shaft_forward  = torque_forward/(2*np.pi*rps_forward)*0.00134102                                # converting W to hp   
                                                                                                               
            power_out_lift          = thrust_lift*segment.conditions.freestream.velocity[i,0]*0.00134102       # converting W to hp   
                                    
            prop_omega_forward      = segment.conditions.propulsion.rpm_forward[i,0]*0.104719755  
            prop_omega_lift         = segment.conditions.propulsion.rpm_lift[i,0]*0.104719755 

            ts_forward              = prop_omega_forward*prop_radius_ft_forward 
            ts_lift                 = prop_omega_lift*prop_radius_ft_lift 

            V_inf          =  segment.conditions.freestream.velocity[i,0]
            result_mat[j]  = np.array([time, CLift,CDrag ,AoA,l_d , eta_l ,
                                       eta_f,energy, volts,volts_oc ,current_lift, battery_amp_hr, C_rating ,altitude  ,
                                      rpm_lift,rps_lift,thrust_lift, torque_lift,effp_lift,effm_lift ,             
                                      rpm_forward,thrust_forward, torque_forward, effp_forward, effm_forward ,             
                                      spec_power_in_bat,power_in_shaft_lift,power_in_shaft_forward,power_out_lift ,             
                                      prop_omega_forward, prop_omega_lift, ts_forward, ts_lift, V_inf,current_forward,
                                      power_in_bat_forward,power_in_bat_lift])


            j += 1                                                                 

    np.save('Cora_New_Results.npy',result_mat)

    return    
 
  

# ------------------------------------------------------------------
#   Electronic Conditions
# ------------------------------------------------------------------
def plot_battery_pack_conditions(results, line_color = 'bo-', save_figure = False, save_filename = "Electronic_Conditions"):
    axis_font = {'size':'14'} 
    fig = plt.figure(save_filename)
    fig.set_size_inches(12, 10)
    for i in range(len(results.segments)):  
    
        time     = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        power    = results.segments[i].conditions.propulsion.battery_power_draw[:,0] 
        energy   = results.segments[i].conditions.propulsion.battery_energy[:,0] 
        volts    = results.segments[i].conditions.propulsion.voltage_under_load[:,0] 
        volts_oc = results.segments[i].conditions.propulsion.voltage_open_circuit[:,0]     
        current = results.segments[i].conditions.propulsion.current[:,0]      
        battery_amp_hr = (energy*0.000277778)/volts
        C_rating   = current/battery_amp_hr
        
        axes = fig.add_subplot(2,2,1)
        axes.plot(time, -power, line_color)
        axes.set_ylabel('Battery Power (Watts)',axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')   
        axes.grid(True)       
    
        axes = fig.add_subplot(2,2,2)
        axes.plot(time, energy*0.000277778, line_color)
        axes.set_ylabel('Battery Energy (W-hr)',axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')       
        axes.grid(True)   
    
        axes = fig.add_subplot(2,2,3)
        axes.plot(time, volts, 'bo-',label='Under Load')
        axes.plot(time,volts_oc, 'ks--',label='Open Circuit')
        axes.set_xlabel('Time (mins)',axis_font)
        axes.set_ylabel('Battery Voltage (Volts)',axis_font)  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')   
        if i == 0:
            axes.legend(loc='upper right')          
        axes.grid(True)         
        
        axes = fig.add_subplot(2,2,4)
        axes.plot(time, C_rating, line_color)
        axes.set_xlabel('Time (mins)',axis_font)
        axes.set_ylabel('C-Rating (C)',axis_font)  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')      
        axes.grid(True)
 
    if save_figure:
        plt.savefig(save_filename + ".png")       
        
    return
 

# ------------------------------------------------------------------
#   Lift-Cruise Network
# ------------------------------------------------------------------
def plot_lift_cruise_network(results, line_color = 'bo-', save_figure = False, save_filename = "Lift_Cruise_Network"):
    axis_font = {'size':'14'} 
    # ------------------------------------------------------------------
    #   Electronic Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Lift_Cruise_Electric_Conditions")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)):          
        time           = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        eta            = results.segments[i].conditions.propulsion.throttle[:,0]
        eta_l          = results.segments[i].conditions.propulsion.throttle_lift[:,0]
        energy         = results.segments[i].conditions.propulsion.battery_energy[:,0]*0.000277778
        specific_power = results.segments[i].conditions.propulsion.battery_specfic_power[:,0]
        volts          = results.segments[i].conditions.propulsion.voltage_under_load[:,0] 
        volts_oc       = results.segments[i].conditions.propulsion.voltage_open_circuit[:,0]  
                    
        axes = fig.add_subplot(2,2,1)
        axes.plot(time, eta, 'bo-',label='Forward Motor')
        axes.plot(time, eta_l, 'r^-',label='Lift Motors')
        axes.set_ylabel('Throttle')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        axes.grid(True)       
        plt.ylim((0,1))
        if i == 0:
            axes.legend(loc='upper center')         
    
        axes = fig.add_subplot(2,2,2)
        axes.plot(time, energy, 'bo-')
        axes.set_ylabel('Battery Energy (W-hr)')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')       
        axes.grid(True)   
    
        axes = fig.add_subplot(2,2,3)
        axes.plot(time, volts, 'bo-',label='Under Load')
        axes.plot(time,volts_oc, 'ks--',label='Open Circuit')
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Battery Voltage (Volts)')  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')       
        axes.grid(True)
        if i == 0:
            axes.legend(loc='upper center')                
        
        axes = fig.add_subplot(2,2,4)
        axes.plot(time, specific_power, 'bo-') 
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Specific Power')  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')      
        axes.grid(True)   
        
        
    
    if save_figure:
        plt.savefig("Lift_Cruise_Electric_Conditions.png")
    
   
    # ------------------------------------------------------------------
    #   Propulsion Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Prop-Rotor Network")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)):          
        time   = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        prop_rpm    = results.segments[i].conditions.propulsion.rpm_forward [:,0] 
        prop_thrust = results.segments[i].conditions.frames.body.thrust_force_vector[:,0]
        prop_torque = results.segments[i].conditions.propulsion.motor_torque_forward[:,0]
        prop_effp   = results.segments[i].conditions.propulsion.propeller_efficiency_forward[:,0]
        prop_effm   = results.segments[i].conditions.propulsion.motor_efficiency_forward[:,0]
        prop_Cp     = results.segments[i].conditions.propulsion.propeller_power_coefficient[:,0]
        rotor_rpm    = results.segments[i].conditions.propulsion.rpm_lift[:,0] 
        rotor_thrust = -results.segments[i].conditions.frames.body.thrust_force_vector[:,2]
        rotor_torque = results.segments[i].conditions.propulsion.motor_torque_lift[:,0]
        rotor_effp   = results.segments[i].conditions.propulsion.propeller_efficiency_lift[:,0]
        rotor_effm   = results.segments[i].conditions.propulsion.motor_efficiency_lift[:,0] 
        rotor_Cp = results.segments[i].conditions.propulsion.propeller_power_coefficient_lift[:,0]        
    
        axes = fig.add_subplot(2,3,1)
        axes.plot(time, prop_rpm, 'bo-')
        axes.plot(time, rotor_rpm, 'r^-')
        axes.set_ylabel('RPM')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        axes.grid(True)       
    
        axes = fig.add_subplot(2,3,2)
        axes.plot(time, prop_thrust, 'bo-')
        axes.plot(time, rotor_thrust, 'r^-')
        axes.set_ylabel('Thrust (N)')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')       
        axes.grid(True)   
    
        axes = fig.add_subplot(2,3,3)
        axes.plot(time, prop_torque, 'bo-' )
        axes.plot(time, rotor_torque, 'r^-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Torque (N-m)')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')      
        axes.grid(True)   
    
        axes = fig.add_subplot(2,3,4)
        axes.plot(time, prop_effp, 'bo-' )
        axes.plot(time, rotor_effp, 'r^-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel(r'Propeller Efficiency, $\eta_{propeller}$')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')      
        axes.grid(True)           
        plt.ylim((0,1))
    
        axes = fig.add_subplot(2,3,5)
        axes.plot(time, prop_effm, 'bo-' )
        axes.plot(time, rotor_effm, 'r^-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel(r'Motor Efficiency, $\eta_{motor}$')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')      
        axes.grid(True)         
        plt.ylim((0,1))
    
        axes = fig.add_subplot(2,3,6)
        axes.plot(time, prop_Cp, 'bo-' )
        axes.plot(time, rotor_Cp, 'r^-'  )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Power Coefficient')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  
        axes.grid(True) 
        
    if save_figure:
        plt.savefig("Propulsor Network.png")
            
    # ------------------------------------------------------------------
    #   Propulsion Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Rotor")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)):          
        time   = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        rpm    = results.segments[i].conditions.propulsion.rpm_lift [:,0] 
        thrust = results.segments[i].conditions.frames.body.thrust_force_vector[:,2]
        torque = results.segments[i].conditions.propulsion.motor_torque_lift[:,0]
        effp   = results.segments[i].conditions.propulsion.propeller_efficiency_lift[:,0]
        effm   = results.segments[i].conditions.propulsion.motor_efficiency_lift[:,0] 
        Cp     = results.segments[i].conditions.propulsion.propeller_power_coefficient_lift[:,0]
    
        axes = fig.add_subplot(2,3,1)
        axes.plot(time, rpm, 'r^-')
        axes.set_ylabel('RPM')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        axes.grid(True)       
    
        axes = fig.add_subplot(2,3,2)
        axes.plot(time, -thrust, 'r^-')
        axes.set_ylabel('Thrust (N)')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')       
        axes.grid(True)   
    
        axes = fig.add_subplot(2,3,3)
        axes.plot(time, torque, 'r^-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Torque (N-m)')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')      
        axes.grid(True)   
    
        axes = fig.add_subplot(2,3,4)
        axes.plot(time, effp, 'r^-',label= r'$\eta_{rotor}$' ) 
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel(r'Propeller Efficiency $\eta_{rotor}$')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')   
        #if i == 0:
            #axes.legend(loc='upper center')   
        axes.grid(True)           
        plt.ylim((0,1))
    
        axes = fig.add_subplot(2,3,5)
        axes.plot(time, effm, 'r^-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel(r'Motor Efficiency $\eta_{mot}$')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')      
        plt.ylim((0,1))
        axes.grid(True)  
    
        axes = fig.add_subplot(2,3,6)
        axes.plot(time, Cp , 'r^-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Power Coefficient')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')    
        axes.grid(True)           
    
    if save_figure:
        plt.savefig("Rotor.png")  
        
        
    # ------------------------------------------------------------------
    #   Propulsion Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Propeller")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)):          
        time   = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        rpm    = results.segments[i].conditions.propulsion.rpm_forward [:,0] 
        thrust = results.segments[i].conditions.frames.body.thrust_force_vector[:,0]
        torque = results.segments[i].conditions.propulsion.motor_torque_forward
        effp   = results.segments[i].conditions.propulsion.propeller_efficiency_forward[:,0]
        effm   = results.segments[i].conditions.propulsion.motor_efficiency_forward[:,0]
        Cp     = results.segments[i].conditions.propulsion.propeller_power_coefficient[:,0]
    
        axes = fig.add_subplot(2,3,1)
        axes.plot(time, rpm, 'bo-')
        axes.set_ylabel('RPM')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        axes.grid(True)       
    
        axes = fig.add_subplot(2,3,2)
        axes.plot(time, thrust, 'bo-')
        axes.set_ylabel('Thrust (N)')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')       
        axes.grid(True)   
    
        axes = fig.add_subplot(2,3,3)
        axes.plot(time, torque, 'bo-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Torque (N-m)')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')      
        axes.grid(True)   
    
        axes = fig.add_subplot(2,3,4)
        axes.plot(time, effp, 'bo-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel(r'Propeller Efficiency $\eta_{propeller}$')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')      
        axes.grid(True)           
        plt.ylim((0,1))
    
        axes = fig.add_subplot(2,3,5)
        axes.plot(time, effm, 'bo-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel(r'Motor Efficiency $\eta_{motor}$')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')      
        axes.grid(True)         
        plt.ylim((0,1))
    
        axes = fig.add_subplot(2,3,6)
        axes.plot(time, Cp, 'bo-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Power Coefficient')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  
        axes.grid(True) 
        
    if save_figure:
        plt.savefig("Cruise_Propulsor.png") 
       
    # ------------------------------------------------------------------
    #   Propulsion Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Tip_Mach") 
    for i in range(len(results.segments)):          
        time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min 
        rtm  = results.segments[i].conditions.propulsion.propeller_tip_mach_lift[:,0]
        ptm  = results.segments[i].conditions.propulsion.propeller_tip_mach_forward[:,0] 
        
        axes = fig.add_subplot(1,1,1)
        axes.plot(time, ptm, 'bo-',label='Propeller')
        axes.plot(time, rtm, 'r^-',label='Rotor')
        axes.set_ylabel('Mach')
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        axes.grid(True)       
        
        if i == 0:
            axes.legend(loc='upper center')     
    
    if save_figure:
        plt.savefig("Tip_Mach.png")  
        
        
    return

if __name__ == '__main__': 
    main()    
    plt.show()