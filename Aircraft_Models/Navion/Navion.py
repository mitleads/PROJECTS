# Navion.py
# 
# Created: Dec 2021, M. Clarke

""" setup file for a mission with a Navion
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Units , Data  
from SUAVE.Plots.Performance.Mission_Plots                       import *  
from SUAVE.Plots.Geometry                                        import *  
from SUAVE.Methods.Propulsion                                    import propeller_design 
import vsp 
from SUAVE.Input_Output.OpenVSP.vsp_write import write 
from SUAVE.Methods.Propulsion import propeller_design 

import numpy as np
import pylab as plt
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
    write(vehicle, 'Navion') 
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
    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Navion' 

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------

    # mass properties
    vehicle.mass_properties.max_takeoff   = 1335.76 
    vehicle.mass_properties.takeoff       = 1335.76   
    vehicle.mass_properties.moments_of_inertia.tensor = np.array([[164627.7,0.0,0.0],[0.0,471262.4,0.0],[0.0,0.0,554518.7]])
    vehicle.mass_properties.center_of_gravity = [[2.239696797,0,-0.131189711 ]]
    vehicle.envelope.ultimate_load        = 5.7
    vehicle.envelope.limit_load           = 3.8
    vehicle.reference_area                = 17.112 
    vehicle.passengers                    = 2
      
    
    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------    
    wing                                  = SUAVE.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing' 
    wing.sweeps.leading_edge              = 2.996 * Units.degrees 
    wing.thickness_to_chord               = 12
    wing.areas.reference                  = 17.112
    wing.chords.mean_aerodynamic          = 1.74 
    wing.taper                            = 0.54 
    wing.aspect_ratio                     = 6.04  
    wing.spans.projected                  = 10.166
    wing.chords.root                      = 2.1944 
    wing.chords.tip                       = 1.1850
    wing.twists.root                      = 2 * Units.degrees  
    wing.twists.tip                       = -1 * Units.degrees   
    wing.dihedral                         = 7.5 * Units.degrees   
    wing.origin                           = [[1.652555594, 0.,-0.6006666]]
    wing.aerodynamic_center               = [1.852555594, 0., 6006666 ] # INCORRECT 
    wing.vertical                         = False
    wing.symmetric                        = True
    wing.high_lift                        = True 
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0  
    
    tip_airfoil                           = SUAVE.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = 'Airfoils/NACA_6410.txt' 
 
    root_airfoil                          = SUAVE.Components.Airfoils.Airfoil()
    root_airfoil.coordinate_file          = 'Airfoils/NACA_4415.txt' 
    
    # Wing Segments
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0
    segment.twist                         = 2 * Units.degrees   
    segment.root_chord_percent            = 1 
    segment.dihedral_outboard             = 7.5 * Units.degrees  
    segment.sweeps.quarter_chord          = 2.996 * Units.degrees  
    segment.thickness_to_chord            = 0.10  
    segment.append_airfoil(root_airfoil)
    wing.append_segment(segment)

    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1
    segment.twist                         = -1 * Units.degrees
    segment.root_chord_percent            = wing.taper
    segment.dihedral_outboard             = 0 
    segment.sweeps.quarter_chord          = 0 
    segment.thickness_to_chord            = 0.15
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)
     
    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------       
    wing                                  = SUAVE.Components.Wings.Wing()
    wing.tag                              = 'horizontal_stabilizer'  
    wing.sweeps.leading_edge              = 6 * Units.degrees 
    wing.thickness_to_chord               = 12
    wing.areas.reference                  = 4  
    wing.aspect_ratio                     = 4 
    wing.spans.projected                  = 4.0
    wing.chords.root                      = 1.2394
    wing.chords.mean_aerodynamic          = 1.0484
    wing.chords.tip                       = 0.8304
    wing.taper                            = 0.67 
    wing.twists.root                      = 0 * Units.degrees  
    wing.twists.tip                       = 0 * Units.degrees   
    wing.origin                           = [[ 6.54518625 , 0., 0.203859697]]
    wing.aerodynamic_center               = [[ 6.545186254 + 0.25*wing.spans.projected, 0., 0.203859697]] 
    wing.vertical                         = False
    wing.winglet_fraction                 = 0.0  
    wing.symmetric                        = True
    wing.high_lift                        = False 
    wing.dynamic_pressure_ratio           = 0.9 


    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------ 
    wing                                  = SUAVE.Components.Wings.Wing()
    wing.tag                              = 'vertical_stabilizer'   
    wing.sweeps.leading_edge              = 20 * Units.degrees 
    wing.thickness_to_chord               = 12.5
    wing.areas.reference                  = 1.163 
    wing.spans.projected                  = 1.4816
    wing.chords.root                      = 1.2176
    wing.chords.tip                       = 1.2176
    wing.chords.tip                       = 0.5870 
    wing.aspect_ratio                     = 1.8874 
    wing.taper                            = 0.4820 
    wing.chords.mean_aerodynamic          = 0.9390 
    wing.twists.root                      = 0 * Units.degrees  
    wing.twists.tip                       = 0 * Units.degrees   
    wing.origin                           = [[ 7.127369987, 0., 0.303750948]]
    wing.aerodynamic_center               = [ 7.49778005775, 0., 0.67416101875] 
    wing.vertical                         = True 
    wing.symmetric                        = False
    wing.t_tail                           = False
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0  
    
    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------
    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage'
    fuselage.seats_abreast                      = 2
    fuselage.lengths.total                      = 8.349950916 
    fuselage.width                              = 1.22028016 
    fuselage.heights.maximum                    = 1.634415138  
    fuselage.areas.wetted                       = 12. # ESTIMATED 
    fuselage.areas.front_projected              = fuselage.width*fuselage.heights.maximum
    fuselage.effective_diameter                 = 1.22028016 

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_0'
    segment.percent_x_location                  = 0
    segment.percent_z_location                  = 0
    segment.height                              = 0.529255748
    segment.width                               = 0.575603849
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_1'
    segment.percent_x_location                  =  0.028527593
    segment.percent_z_location                  =  0
    segment.height                              =  0.737072721
    segment.width                               =  0.921265952 
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_2'
    segment.percent_x_location                  = 0.187342754 
    segment.percent_z_location                  = 0 
    segment.height                              = 1.174231852 
    segment.width                               = 1.196956212
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_3'
    segment.percent_x_location                  = 0.242034847 
    segment.percent_z_location                  = 0.011503528 
    segment.height                              = 1.450221906 
    segment.width                               = 1.173932059 
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_4'
    segment.percent_x_location                  = 0.296715183 
    segment.percent_z_location                  = 0.015984303 
    segment.height                              = 1.634415138 
    segment.width                               = 1.22028016 
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_5'
    segment.percent_x_location                  = 0.510275342 
    segment.percent_z_location                  = -0.005
    segment.height                              = 1.082135236 
    segment.width                               = 1.013062774 
    fuselage.Segments.append(segment)

    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_6'
    segment.percent_x_location                  = 0.833284347 
    segment.percent_z_location                  = 0.014138855 
    segment.height                              = 0.621652157 
    segment.width                               = 0.414134978
    fuselage.Segments.append(segment)
 
    # Segment
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_7'
    segment.percent_x_location                  = 1
    segment.percent_z_location                  = 0.018978667 
    segment.height                              = 0.092096616 
    segment.width                               = 0.046048308 
    fuselage.Segments.append(segment)
    
    # add to vehicle
    vehicle.append_component(fuselage)
 
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
    prop.tip_radius              = 2.14/2
    prop.hub_radius              = 8.     * Units.inches
    prop.design_Cl               = 0.8
    prop.design_altitude         = 12000. * Units.feet
    prop.design_power            = .64 * 180. * Units.horsepower
    prop.variable_pitch          = True
    prop.origin                  = [[-0.1, 0, 0]] 
    prop.airfoil_geometry        =  ['Airfoils/NACA_4412.txt'] 
    prop.airfoil_polars          = [['Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                     'Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                     'Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                     'Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                     'Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]

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
    #plot_vehicle(vehicle, elevation_angle = 0 ,azimuthal_angle = 270, axis_limits = 5.2 ,plot_control_points = False) 
    #plt.show()  
    

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
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,line_style='bo-'):
    plot_flight_conditions(results, line_style) 
        
    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style) 
    
    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results, line_style)  
    
    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results, line_style) 

    # Plot Aircraft Stability
    plot_stability_coefficients(results,line_style) 
     
    
    # Plot Propeller Conditions 
    plot_propeller_conditions(results, line_style) 
     
    # Plot propeller Disc and Power Loading
    plot_disc_power_loading(results, line_style)   
     
    
    # Plot noise level
    #plot_ground_noise_levels(results)
     
    
    # Plot noise contour
    #plot_flight_profile_noise_contours(results) 
        
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