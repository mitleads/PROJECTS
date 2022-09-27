# Cessna_172.py
# 
# Created:  Sep 2016, E. Botero
# Modified: Sep 2016, C. Ilario 

""" setup file for a mission with a C172 SP NAV III
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Units

import numpy as np
import pylab as plt

from SUAVE.Core import Data
from SUAVE.Methods.Propulsion import propeller_design

from SUAVE.Input_Output.OpenVSP.vsp_write import write
from SUAVE.Input_Output.OpenVSP.get_vsp_areas import get_vsp_areas
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_energy_and_power, initialize_from_mass
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv
import cProfile, pstats, StringIO
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():

    configs, analyses = full_setup() 

    configs.finalize()
    analyses.finalize()  

    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()

    # plt the old results
    plot_mission(results)


    return


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():

    # vehicle data
    vehicle  = vehicle_setup()
    #write(vehicle, 'Cessna_172') 
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(configs_analyses,vehicle)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    configs, analyses = full_setup() 

    configs.finalize()
    analyses.finalize()

    # weight analysis
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()      

    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()

    # plt the old results
    plot_mission(results)


    return analyses
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
    weights = SUAVE.Analyses.Weights.Weights_Transport()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()    
    #aerodynamics.process.compute.lift.inviscid.settings.spanwise_vortex_density    = 3
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()    
    #stability.settings.spanwise_vortex_density                  = 3
    stability.geometry = vehicle
    analyses.append(stability)

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

    # done!
    return analyses    


# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup(): 
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------        
    vehicle                                     = SUAVE.Vehicle()
    vehicle.tag                                 = 'Cessna_172_SP'
                                                
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff         = 2550. * Units.pounds
    vehicle.mass_properties.takeoff             = 2550. * Units.pounds
    vehicle.mass_properties.max_zero_fuel       = 2550. * Units.pounds
    vehicle.mass_properties.cargo               = 0. 
                                               
    # envelope properties                       
    vehicle.envelope.ultimate_load              = 5.7
    vehicle.envelope.limit_load                 = 3.8
                                                
    cruise_speed                                = 124. * Units.kts
    altitude                                    = 8500. * Units.ft
    atmo                                        = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    freestream                                  = atmo.compute_values (0.)
    freestream0                                 = atmo.compute_values (altitude)
    mach_number                                 = (cruise_speed/freestream.speed_of_sound)[0][0] 
    vehicle.design_dynamic_pressure             = ( .5 *freestream0.density*(cruise_speed*cruise_speed))[0][0]
    vehicle.design_mach_number                  =  mach_number
                                                
    # basic parameters                          
    vehicle.reference_area                      = 174. * Units.feet**2       
    vehicle.passengers                          = 4

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        

    wing                                        = SUAVE.Components.Wings.Main_Wing()
    wing.tag                                    = 'main_wing'    
    wing.sweeps.quarter_chord                   = 0.0 * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.areas.reference                        = 174. * Units.feet**2
    wing.spans.projected                        = 36.  * Units.feet + 1. * Units.inches
    wing.chords.root                            = 66. * Units.inches
    wing.chords.tip                             = 45. * Units.inches
    wing.chords.mean_aerodynamic                = 58. * Units.inches
    wing.taper                                  = wing.chords.tip/wing.chords.root
    wing.aspect_ratio                           = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root                            = 3.0 * Units.degrees
    wing.twists.tip                             = 1.5 * Units.degrees
    wing.origin                                 = [[80.* Units.inches,0,0]]
    wing.aerodynamic_center                     = [22.* Units.inches,0,0]
    wing.vertical                               = False
    wing.symmetric                              = True
    wing.high_lift                              = True 
    wing.dynamic_pressure_ratio                 = 1.0 
                                          
    # control surfaces -------------------------------------------
    flap                                        = SUAVE.Components.Wings.Control_Surfaces.Flap() 
    flap.tag                                    = 'flap' 
    flap.span_fraction_start                    = 0.15 
    flap.span_fraction_end                      = 0.324    
    flap.deflection                             = 1.0 * Units.deg
    flap.chord_fraction                         = 0.19    
    wing.append_control_surface(flap)           
                                                
    slat                                        = SUAVE.Components.Wings.Control_Surfaces.Slat() 
    slat.tag                                    = 'slat' 
    slat.span_fraction_start                    = 0.324 
    slat.span_fraction_end                      = 0.963     
    slat.deflection                             = 1.0 * Units.deg
    slat.chord_fraction                         = 0.1      
    wing.append_control_surface(slat)  
    
    SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing) 

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        
                                                
    wing                                        = SUAVE.Components.Wings.Wing()
    wing.tag                                    = 'horizontal_stabilizer' 
    wing.sweeps.quarter_chord                   = 0.0 * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.areas.reference                        = 5800. * Units.inches**2
    wing.spans.projected                        = 136.  * Units.inches
    wing.chords.root                            = 55. * Units.inches
    wing.chords.tip                             = 30. * Units.inches
    wing.chords.mean_aerodynamic                = 43. * Units.inches 
    wing.taper                                  = wing.chords.tip/wing.chords.root
    wing.aspect_ratio                           = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees
    wing.origin                                 = [[246.* Units.inches,0,0]]
    wing.aerodynamic_center                     = [20.* Units.inches,0,0]
    wing.vertical                               = False
    wing.symmetric                              = True
    wing.high_lift                              = False 
    wing.dynamic_pressure_ratio                 = 0.9
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing                                        = SUAVE.Components.Wings.Wing()
    wing.tag                                    = 'vertical_stabilizer' 
    wing.sweeps.quarter_chord                   = 25. * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.areas.reference                        = 3500. * Units.inches**2
    wing.spans.projected                        = 73.   * Units.inches
    wing.chords.root                            = 66. * Units.inches
    wing.chords.tip                             = 27. * Units.inches
    wing.chords.mean_aerodynamic                = 48. * Units.inches 
    wing.taper                                  = wing.chords.tip/wing.chords.root
    wing.aspect_ratio                           = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees
    wing.origin                                 = [[237.* Units.inches,0,0]]
    wing.aerodynamic_center                     = [20.* Units.inches,0,0] 
    wing.vertical                               = True 
    wing.symmetric                              = False
    wing.t_tail                                 = False 
    wing.dynamic_pressure_ratio                 = 1.0

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage                                    = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage'
    fuselage.number_coach_seats                 = 4.       
    fuselage.tag                                = 'fuselage'    
    fuselage.differential_pressure              = 8*Units.psi                    # Maximum differential pressure
    fuselage.width                              = 42.         * Units.inches     # Width of the fuselage
    fuselage.heights.maximum                    = 62. * Units.inches    # Height of the fuselage
    fuselage.lengths.total                      = 326.         * Units.inches            # Length of the fuselage
    fuselage.lengths.empennage                  = 161. * Units.inches  
    fuselage.lengths.cabin                      = 105. * Units.inches
    fuselage.lengths.structure                  = fuselage.lengths.total-fuselage.lengths.empennage 
    fuselage.mass_properties.volume             = .4*fuselage.lengths.total*(np.pi/4.)*(fuselage.heights.maximum**2.) #try this as approximation
    fuselage.mass_properties.internal_volume    = .3*fuselage.lengths.total*(np.pi/4.)*(fuselage.heights.maximum**2.)
    fuselage.areas.wetted                       = 30000. * Units.inches**2.
    fuselage.seats_abreast                      = 2.
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2.
    fuselage.lengths.nose                       = 60.  * Units.inches
    fuselage.heights.at_quarter_length          = 62. * Units.inches
    fuselage.heights.at_three_quarters_length   = 62. * Units.inches
    fuselage.heights.at_wing_root_quarter_chord = 23. * Units.inches
    fuselage.areas.front_projected              = fuselage.width* fuselage.heights.maximum
    fuselage.effective_diameter                 = 50. * Units.inches

    # add to vehicle
    vehicle.append_component(fuselage)
    
    # ------------------------------------------------------------------
    #   Landing gear
    # ------------------------------------------------------------------  
    landing_gear                                = SUAVE.Components.Landing_Gear.Landing_Gear()
    main_gear                                   = SUAVE.Components.Landing_Gear.Main_Landing_Gear()
    nose_gear                                   = SUAVE.Components.Landing_Gear.Nose_Landing_Gear()
    main_gear.strut_length                      = 12. * Units.inches #guess based on picture
    nose_gear.strut_length                      = 6. * Units.inches 
                                                
    landing_gear.main                           = main_gear
    landing_gear.nose                           = nose_gear
                                                
    #add to vehicle                             
    vehicle.landing_gear                        = landing_gear
                                                
    # ------------------------------------------------------------------
    #   Fuel
    # ------------------------------------------------------------------    
    # define fuel weight needed to size fuel system
    fuel                                        = SUAVE.Attributes.Propellants.Aviation_Gasoline()
    fuel.mass_properties                        = SUAVE.Components.Mass_Properties() 
    fuel.number_of_tanks                        = 1.
    fuel.origin                                 = wing.origin
    fuel.internal_volume                        = fuel.mass_properties.mass/fuel.density #all of the fuel volume is internal
    fuel.mass_properties.center_of_gravity      = wing.mass_properties.center_of_gravity
    fuel.mass_properties.mass                   = 319 *Units.lbs
    vehicle.fuel                                = fuel

    # ------------------------------------------------------------------
    #   Piston Propeller Network
    # ------------------------------------------------------------------    
    
    # build network
    net                                         = SUAVE.Components.Energy.Networks.Internal_Combustion_Propeller()
    net.tag                                     = 'internal_combustion'
    net.number_of_engines                       = 1.
    net.identical_propellers                    = True
                                                
    # the engine                    
    engine                                  = SUAVE.Components.Energy.Converters.Internal_Combustion_Engine()
    engine.sea_level_power                  = 180. * Units.horsepower
    engine.flat_rate_altitude               = 0.0
    engine.rated_speed                      = 2700. * Units.rpm
    engine.power_specific_fuel_consumption  = 0.52 
    net.engines.append(engine)
    
    # the prop
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.number_of_blades        = 2.0
    prop.freestream_velocity     = 119.   * Units.knots
    prop.angular_velocity        = 2650.  * Units.rpm
    prop.tip_radius              = 76./2. * Units.inches
    prop.hub_radius              = 8.     * Units.inches
    prop.design_Cl               = 0.8
    prop.design_altitude         = 12000. * Units.feet
    prop.design_power            = .64 * 180. * Units.horsepower
    prop.variable_pitch          = True

    prop.airfoil_geometry        =  ['../Vehicles/Airfoils/NACA_4412.txt'] 
    prop.airfoil_polars          = [['../Vehicles//Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                     '../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                     '../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                     '../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                     '../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]

    prop.airfoil_polar_stations  = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]      
    prop                         = propeller_design(prop)   
    
    net.propellers.append(prop)
     
    
    # add the network to the vehicle
    vehicle.append_component(net) 

    #find uninstalled avionics weight
    Wuav                                        = 2. * Units.lbs
    avionics                                    = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.mass_properties.uninstalled        = Wuav
    vehicle.avionics                            = avionics     
   
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle


# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config) 
    
    # done!
    return configs

# ----------------------------------------------------------------------
#   Sizing for the Vehicle Configs
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,line_style='bo-'):

        
    return
 
# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses,vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    #airport
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
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip
    base_segment.process.iterate.unknowns.network            = vehicle.propulsors.network.unpack_unknowns
    base_segment.process.iterate.residuals.network           = vehicle.propulsors.network.residuals
    base_segment.state.unknowns.propeller_power_coefficient  = 0.005 * ones_row(1) 
    base_segment.state.unknowns.battery_voltage_under_load   = vehicle.propulsors.network.battery.max_voltage * ones_row(1)  
    base_segment.state.residuals.network                     = 0. * ones_row(2)       
    
    ## ------------------------------------------------------------------
    ##   First Climb Segment: constant Speed, constant rate segment 
    ## ------------------------------------------------------------------

    #segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    #segment.tag = "climb_1"

    #segment.analyses.extend( analyses.takeoff )

    #segment.altitude_start = 0.0   * Units.km
    #segment.altitude_end   = 1000. * Units.feet
    #segment.air_speed      = 73.5  * Units['knots']
    #segment.climb_rate     = 695.  * Units['ft/min']
    
    ## add to misison
    #mission.append_segment(segment)
   
    # ------------------------------------------------------------------
    #   First Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.cruise )

    segment.altitude  = 8000. * Units.feet
    segment.air_speed = 92.   * Units['knots']
    segment.distance  = 100.   * Units.nautical_mile

    # add to misison
    mission.append_segment(segment)    

    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission

def missions_setup(base_mission):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission


    # done!
    return missions  



if __name__ == '__main__': 
    main()    
    plt.show()