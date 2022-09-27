# ECTOL_2P.py
#
# Created: Feb 2020, M. Clarke
#          Sep 2020, M. Clarke 

""" setup file for the X57-Maxwell Electric Aircraft 
"""
 
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Units 
import numpy as np   
import matplotlib.pyplot        as plt
import os
import pickle
from copy import deepcopy
import time 
from SUAVE.Core                                                  import Data 
from SUAVE.Plots.Mission_Plots                                   import *  
from SUAVE.Plots.Geometry_Plots                                  import * 
from SUAVE.Components.Energy.Networks.Battery_Propeller          import Battery_Propeller
from SUAVE.Methods.Propulsion                                    import propeller_design  
from SUAVE.Methods.Power.Battery.Sizing                          import initialize_from_circuit_configuration 
from SUAVE.Methods.Propulsion.electric_motor_sizing              import size_optimal_motor
from SUAVE.Methods.Weights.Correlations.Propulsion               import nasa_motor
from SUAVE.Methods.Weights.Buildups.eVTOL.empty                  import empty as evtol_empty 

#import vsp 
#from SUAVE.Input_Output.OpenVSP.vsp_write import write 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():   

    ti = time.time()

    simulated_days  = 1
    flights_per_day = 1 
    aircraft_range  = 80 * Units.miles 
    reserve_segment = True 
    control_points  = 16 
    recharge_battery = False 
    
    # build the vehicle, configs, and analyses
    configs, analyses = full_setup(simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery )

    configs.finalize()
    analyses.finalize()  

    # weight analysis
    evtol_breakdown = evtol_empty(configs.base,contingency_factor=1.1)
    print(evtol_breakdown)      
    
    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate() 
     
    tf = time.time()
    print ('time taken: '+ str(round(((tf-ti)/60),3)) + ' mins')     
    
    # save results  
    save_results(results)
    
    # plot the results
    plot_results(results) 
     
    return


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup(simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery,): 

    # vehicle data
    vehicle  = vehicle_setup() 
    #write(vehicle, "ECTOL_2P")  
    
    # Set up configs
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs,)

    # mission analyses
    mission  = mission_setup(configs_analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses


def base_analysis(vehicle,):

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
    aerodynamics.settings.drag_coefficient_increment = 0.0000 
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Noise Analysis
    noise = SUAVE.Analyses.Noise.Fidelity_One()   
    noise.geometry = vehicle 
    analyses.append(noise)

    # ------------------------------------------------------------------
    #  Energy
    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.networks 
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
    vehicle.tag = 'ECTOL_2P'    
 
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    
    # mass properties
    vehicle.mass_properties.max_takeoff   = 1800
    vehicle.mass_properties.takeoff       = 1800
    vehicle.mass_properties.cargo         = 0.   
    vehicle.envelope.ultimate_load        = 5.7
    vehicle.envelope.limit_load           = 3.8 
    vehicle.reference_area                = 14.76
    vehicle.passengers                    = 6
    vehicle.systems.control               = "fully powered"
    vehicle.systems.accessories           = "commuter"    
    
    cruise_speed                          = 175.*Units['mph']    
    altitude                              = 2500. * Units.ft
    atmo                                  = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    freestream                            = atmo.compute_values (0.)
    freestream0                           = atmo.compute_values (altitude)
    mach_number                           = (cruise_speed/freestream.speed_of_sound)[0][0] 
    vehicle.design_dynamic_pressure       = ( .5 *freestream0.density*(cruise_speed*cruise_speed))[0][0]
    vehicle.design_mach_number            =  mach_number
    
    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------    
    wing                                  = SUAVE.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing' 
    wing.sweeps.quarter_chord             = 0.0 * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 14.76
    wing.spans.projected                  = 11.4 
    wing.chords.root                      = 1.46
    wing.chords.tip                       = 0.92
    wing.chords.mean_aerodynamic          = 1.19
    wing.taper                            = wing.chords.root/wing.chords.tip 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[2.93, 0., 1.01]]
    wing.aerodynamic_center               = [3., 0., 1.01] 
    wing.vertical                         = False
    wing.symmetric                        = True
    wing.high_lift                        = True 
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0  
    airfoil                               = SUAVE.Components.Airfoils.Airfoil()
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator    
    airfoil.coordinate_file               = rel_path + '/Airfoils/NACA_63_412.txt'
    
    cg_x = wing.origin[0][0] + 0.25*wing.chords.mean_aerodynamic
    cg_z = wing.origin[0][2] - 0.2*wing.chords.mean_aerodynamic
    vehicle.mass_properties.center_of_gravity = [[cg_x,   0.  ,  cg_z ]]  # SOURCE: Design and aerodynamic analysis of a twin-engine commuter aircraft
    
    # Wing Segments
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.0 
    segment.twist                         = 0. * Units.degrees   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)
    
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'outboard'
    segment.percent_span_location         = 0.5438
    segment.twist                         = 0.* Units.degrees 
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0. 
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12 
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)
    
    # Wing Segments
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'winglet_1'
    segment.percent_span_location         = 0.98
    segment.twist                         = 0.  * Units.degrees 
    segment.root_chord_percent            = 0.630
    segment.dihedral_outboard             = 75. * Units.degrees 
    segment.sweeps.quarter_chord          = 82. * Units.degrees 
    segment.thickness_to_chord            = 0.12 
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)
    
    
    # Wing Segments
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'winglet_2'
    segment.percent_span_location         = 0.99
    segment.twist                         = 0.  * Units.degrees 
    segment.root_chord_percent            = 0.25
    segment.dihedral_outboard             = 75. * Units.degrees 
    segment.sweeps.quarter_chord          = 65. * Units.degrees 
    segment.thickness_to_chord            = 0.12 
    segment.append_airfoil(airfoil)
    wing.append_segment(segment) 
    
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.
    segment.twist                         = 0. * Units.degrees 
    segment.root_chord_percent            = 0.12
    segment.dihedral_outboard             = 0.
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)    
    
    # add to vehicle
    vehicle.append_component(wing)
    
    
    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------       
    wing                                  = SUAVE.Components.Wings.Wing()
    wing.tag                              = 'horizontal_stabilizer' 
    wing.sweeps.quarter_chord             = 0.0 * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 2.540 
    wing.spans.projected                  = 3.3  * Units.meter 
    wing.sweeps.quarter_chord             = 0 * Units.deg 
    wing.chords.root                      = 0.769 * Units.meter 
    wing.chords.tip                       = 0.769 * Units.meter 
    wing.chords.mean_aerodynamic          = 0.769 * Units.meter  
    wing.taper                            = 1. 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[7.7, 0., 0.25]]
    wing.aerodynamic_center               = [7.8, 0., 0.25] 
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
    wing.sweeps.quarter_chord             = 25. * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 2.258 * Units['meters**2']  
    wing.spans.projected                  = 1.854   * Units.meter  
    wing.chords.root                      = 1.6764 * Units.meter 
    wing.chords.tip                       = 0.6858 * Units.meter 
    wing.chords.mean_aerodynamic          = 1.21 * Units.meter 
    wing.taper                            = wing.chords.root/wing.chords.tip 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[6.75 ,0,  0.623]]
    wing.aerodynamic_center               = [0.508 ,0,0]  
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
    fuselage                                    = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 2. 
    fuselage.seat_pitch                         = 3.  
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2. 
    fuselage.lengths.nose                       = 60.  * Units.inches
    fuselage.lengths.tail                       = 161. * Units.inches
    fuselage.lengths.cabin                      = 105. * Units.inches
    fuselage.lengths.total                      = 332.2* Units.inches
    fuselage.lengths.fore_space                 = 0.
    fuselage.lengths.aft_space                  = 0.   
    fuselage.width                              = 42. * Units.inches 
    fuselage.heights.maximum                    = 62. * Units.inches
    fuselage.heights.at_quarter_length          = 62. * Units.inches
    fuselage.heights.at_three_quarters_length   = 62. * Units.inches
    fuselage.heights.at_wing_root_quarter_chord = 23. * Units.inches 
    fuselage.areas.side_projected               = 8000.  * Units.inches**2.
    fuselage.areas.wetted                       = 30000. * Units.inches**2.
    fuselage.areas.front_projected              = 42.* 62. * Units.inches**2. 
    fuselage.effective_diameter                 = 50. * Units.inches
    
    
    # Segment  
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment() 
    segment.tag                                 = 'segment_0'  
    segment.percent_x_location                  = 0 
    segment.percent_z_location                  = 0 
    segment.height                              = 0.01 
    segment.width                               = 0.01 
    fuselage.Segments.append(segment)             
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_1'   
    segment.percent_x_location                  = 0.007279116466
    segment.percent_z_location                  = 0.002502014453
    segment.height                              = 0.1669064748
    segment.width                               = 0.2780205877
    fuselage.Segments.append(segment)    
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.01941097724 
    segment.percent_z_location                  = 0.001216095397 
    segment.height                              = 0.3129496403 
    segment.width                               = 0.4365777215 
    fuselage.Segments.append(segment)         
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.06308567604
    segment.percent_z_location                  = 0.007395489231
    segment.height                              = 0.5841726619
    segment.width                               = 0.6735119903
    fuselage.Segments.append(segment)      
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.1653761217 
    segment.percent_z_location                  = 0.02891281352 
    segment.height                              = 1.064028777 
    segment.width                               = 1.067200529 
    fuselage.Segments.append(segment)  
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_5'    
    segment.percent_x_location                  = 0.2426372155 
    segment.percent_z_location                  = 0.04214148761 
    segment.height                              = 1.293766653 
    segment.width                               = 1.183058255 
    fuselage.Segments.append(segment)  
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_6'    
    segment.percent_x_location                  = 0.2960174029  
    segment.percent_z_location                  = 0.04705241831  
    segment.height                              = 1.377026712  
    segment.width                               = 1.181540054  
    fuselage.Segments.append(segment)  
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.3809404284 
    segment.percent_z_location                  = 0.05313580461 
    segment.height                              = 1.439568345 
    segment.width                               = 1.178218989 
    fuselage.Segments.append(segment)    
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.5046854083 
    segment.percent_z_location                  = 0.04655492473 
    segment.height                              = 1.29352518 
    segment.width                               = 1.054390707 
    fuselage.Segments.append(segment)   
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_9'   
    segment.percent_x_location                  = 0.6454149933 
    segment.percent_z_location                  = 0.03741966266 
    segment.height                              = 0.8971223022 
    segment.width                               = 0.8501926505   
    fuselage.Segments.append(segment)  
      
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_10'   
    segment.percent_x_location                  = 0.985107095 
    segment.percent_z_location                  = 0.04540283436 
    segment.height                              = 0.2920863309 
    segment.width                               = 0.2012565415  
    fuselage.Segments.append(segment)         
       
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_11'   
    segment.percent_x_location                  = 1 
    segment.percent_z_location                  = 0.04787575562  
    segment.height                              = 0.1251798561 
    segment.width                               = 0.1206021048 
    fuselage.Segments.append(segment)             
    
    # add to vehicle
    vehicle.append_component(fuselage)   
    
    # ---------------------------------------------------------------------------------------------
    #   Landing gear                                                   
    # ---------------------------------------------------------------------------------------------  
    landing_gear                                = SUAVE.Components.Landing_Gear.Landing_Gear()
    main_gear                                   = SUAVE.Components.Landing_Gear.Main_Landing_Gear()
    nose_gear                                   = SUAVE.Components.Landing_Gear.Nose_Landing_Gear()
    main_gear.strut_length                      = 12. * Units.inches # guess based on picture
    nose_gear.strut_length                      = 6. * Units.inches  
    landing_gear.main                           = main_gear
    landing_gear.nose                           = nose_gear 
    vehicle.landing_gear                        = landing_gear
    
    #---------------------------------------------------------------------------------------------
    # DEFINE PROPELLER
    #---------------------------------------------------------------------------------------------
    # build network    
    net                              = Battery_Propeller() 
    net.number_of_propeller_engines  = 2.
    net.nacelle_diameter             = 0.65 
    net.engine_length                = 1.6
    net.nacelle_start                = 0.2 
    net.nacelle_end                  = 1.  
    net.areas                        = Data() 
    net.areas.wetted                 = net.engine_length*(2*np.pi*net.nacelle_diameter/2)     
    
    # Component 1 the ESC     
    esc                              = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency                   = 0.95 # Gundlach for brushless motors
    net.esc                          = esc 
    
    # Component 2 the Propeller 
    propeller                        = SUAVE.Components.Energy.Converters.Propeller()  
    propeller.number_of_blades       = 3
    propeller.freestream_velocity    = 135.*Units['mph']    
    propeller.angular_velocity       = 1300.  * Units.rpm  
    propeller.tip_radius             = 76./2. * Units.inches  
    propeller.hub_radius             = 8.     * Units.inches
    propeller.design_Cl              = 0.8
    propeller.design_altitude        = 2500. * Units.feet 
    propeller.design_thrust          = 1800.     
    propeller.origin                 = [[2.5, 1.63, 1.01]]     
    propeller.rotation               = [-1,1] 
    propeller.symmetry               = True   
    propeller.airfoil_geometry           =  [ rel_path + '/Airfoils/NACA_4412.txt']
    propeller.airfoil_polars             = [[rel_path +'/Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                          rel_path + '/Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                          rel_path + '/Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                          rel_path + '/Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                          rel_path + '/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]
    propeller.airfoil_polar_stations = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]        
    propeller                        = propeller_design(propeller)    
    
     
    prop_left = deepcopy(propeller)
    prop_left.tag = 'propeller_2' 
    prop_left.origin   = [[2.5, -1.63, 1.01]]
    prop_left.rotation = 1
    
    net.propellers.append(propeller)
    net.propellers.append(prop_left)   
       
    # Component 5 the Battery     
    bat                         = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650()   
    bat.cell.surface_area       = (np.pi*bat.cell.height*bat.cell.diameter)  
    bat.pack_config.series      =  120  
    bat.pack_config.parallel    =  80  
    initialize_from_circuit_configuration(bat)

    bat.heat_transfer_efficiency = 0.75 
 
    #bat.module_config.normal_spacing   = 0.03
    #bat.module_config.parallel_spacing = 0.03    
    net.voltage                  = bat.max_voltage
    
    # Here we, are going to assume a battery pack module shape. This step is optional but
    # required for thermal analysis of the pack. We will assume that all cells electrically connected 
    # in series wihtin the module are arranged in one row normal direction to the airflow. Likewise ,
    # all cells electrically in paralllel are arranged in the direction to the cooling fluid  
    
    number_of_modules                = 10
    bat.module_config.total          = int(np.ceil(bat.pack_config.total/number_of_modules))
    bat.module_config.normal_count   = int(np.ceil(bat.module_config.total/bat.pack_config.series))
    bat.module_config.parallel_count = int(np.ceil(bat.module_config.total/bat.pack_config.parallel))
    net.battery                      = bat  
    
    # Component 4 Miscellaneous Systems 
    sys                           = SUAVE.Components.Systems.System()
    sys.mass_properties.mass      = 5 # kg
     
    # Component 5 the Motor
    motor                         = SUAVE.Components.Energy.Converters.Motor() 
    motor.efficiency              = 0.9
    motor.gearbox_efficiency      = 1.  
    motor.nominal_voltage         = bat.max_voltage * 0.75  
    motor.propeller_radius        = propeller.tip_radius    
    motor.no_load_current         = 0.5
    motor.origin                  = [[2.5, 1.63, 1.01]]     
    motor                         = size_optimal_motor(motor,propeller) 
    motor.mass_properties.mass    = nasa_motor(motor.design_torque)  

    # append right motor
    net.propeller_motors.append(motor)
    
    # append left motor 
    motor_left = deepcopy(motor)
    motor_left.origin = [[2.5, -1.63, 1.01]]
    motor_left.rotation = 1
    net.propeller_motors.append(motor_left) 
    
    
    # Component 6 the Payload
    payload                       = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw            = 10. # Watts 
    payload.mass_properties.mass  = 1.0 * Units.kg
    net.payload                   = payload
                                  
    # Component 7 the Avionics    
    avionics                      = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw           = 20. # Watts  
    net.avionics                  = avionics      
    
    # add the solar network to the vehicle
    vehicle.append_component(net)    
    
    motor_origins                   = np.array(propeller.origin) 
    vehicle.wings['main_wing'].motor_spanwise_locations = motor_origins[:,1]/vehicle.wings['main_wing'].spans.projected

 
        
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------
    # plot vehicle 
    #plot_vehicle(vehicle,plot_control_points = False)
    #plt.show()
    
    return vehicle

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs,):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config,)
        analyses[tag] = analysis

    return analyses 

# ---------------------------------------------------------------------
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
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery):   
    
    # Determine Stall Speed 
    m     = vehicle.mass_properties.max_takeoff
    g     = 9.81
    S     = vehicle.reference_area
    atmo  = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    rho   = atmo.compute_values(1000.*Units.feet,0.).density
    CLmax = 1.2 
    Vstall = float(np.sqrt(2.*m*g/(rho*S*CLmax))) 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments 
    
    # base segment
    base_segment                                             = Segments.Segment()
    ones_row     = base_segment.state.ones_row
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip
    base_segment.state.numerics.number_control_points        = control_points
    bat                                                      = vehicle.networks.battery_propeller.battery
    base_segment.charging_SOC_cutoff                         = bat.cell.charging_SOC_cutoff 
    
    for day in range(simulated_days):
        print(' ***********  Day ' + str(day) + ' ***********  ')
        for f_idx in range(flights_per_day): 
            flight_no = f_idx + 1      
            ## ------------------------------------------------------------------
            ##   Departure End of Runway Segment Flight 1 : 
            ## ------------------------------------------------------------------ 
            #segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
            #segment.tag = 'Departure_End_of_Runway'   + "_F_" + str(flight_no) + "_D" + str (day)     
            #segment.analyses.extend( analyses.base ) 
            #segment.battery_energy                                   = vehicle.networks.battery_propeller.battery.max_energy  
            #segment.altitude_start                                   = 0.0 * Units.feet
            #segment.altitude_end                                     = 50.0 * Units.feet
            #segment.air_speed_start                                  = Vstall  
            #segment.air_speed_end                                    = Vstall*1.1           
            #segment.climb_rate                                       = 600 * Units['ft/min']  
            #segment.state.unknowns.throttle                          = 1.0 * ones_row(1)  
            #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.05)  
            #mission.append_segment(segment) 
            
            ## ------------------------------------------------------------------
            ##   Initial Climb Area Segment Flight 1  
            ## ------------------------------------------------------------------ 
            #segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
            #segment.tag = 'Initial_CLimb_Area'  + "_F_" + str(flight_no) + "_D" + str (day)
            #segment.analyses.extend( analyses.base )   
            #segment.altitude_start                                   = 50.0 * Units.feet
            #segment.altitude_end                                     = 500.0 * Units.feet
            #segment.air_speed_start                                  = Vstall*1.1   
            #segment.air_speed_end                                    = Vstall*1.2   
            #segment.climb_rate                                       = 600 * Units['ft/min']  
            #segment.state.unknowns.throttle                          = 0.85 * ones_row(1)  
            #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.05)  
            #mission.append_segment(segment) 
                     
            ## ------------------------------------------------------------------
            ##   Climb Segment Flight 1 
            ## ------------------------------------------------------------------ 
            #segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
            #segment.tag = 'Climb'   + "_F_" + str(flight_no) + "_D" + str (day)      
            #segment.analyses.extend( analyses.base )         
            #segment.altitude_start                                   = 500.0 * Units.feet
            #segment.altitude_end                                     = 2500 * Units.feet 
            #segment.air_speed_start                                  = Vstall*1.2   
            #segment.air_speed_end                                    = 175.* Units['mph']    
            #segment.climb_rate                                       = 500* Units['ft/min']  
            #segment.state.unknowns.throttle                          = 0.85 * ones_row(1)  
            #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.3)  
            #mission.append_segment(segment)  
            
            # ------------------------------------------------------------------
            #   Cruise Segment Flight 1 
            # ------------------------------------------------------------------ 
            segment = Segments.Cruise.Constant_RPM_Constant_Altitude(base_segment)  
            segment.propeller_rpm            = 1500* Units.rpm
            
            #segment = Segments.Cruise.Constant_Throttle_Constant_Altitude(base_segment)  
            #segment.throttle                   = 0.6            
            
            segment.tag = 'Cruise'  + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend(analyses.base) 
            segment.battery_energy                                   = vehicle.networks.battery_propeller.battery.max_energy *0.75
            segment.air_speed_start                                  = 175.* Units['mph']    
            segment.altitude                                         = 2500 * Units.feet 
            segment.air_speed                                        = 175.* Units['mph']        
            cruise_distance                                          = 65045.04038150153  + (aircraft_range - 76.65*Units.miles )   
            segment.distance                                         = cruise_distance  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.12)  
            mission.append_segment(segment)  
            
            ## ------------------------------------------------------------------
            ##   Descent Segment Flight 1   
            ## ------------------------------------------------------------------ 
            #segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
            #segment.tag = 'Descent'   + "_F_" + str(flight_no) + "_D" + str (day)      
            #segment.analyses.extend( analyses.base )       
            #segment.altitude_start                                   = 2500 * Units.feet  
            #segment.altitude_end                                     = 1000.0 * Units.feet
            #segment.air_speed_start                                  = 175.* Units['mph']    
            #segment.air_speed_end                                    = 110 * Units['mph']   
            #segment.climb_rate                                       = -200 * Units['ft/min']  
            #segment.state.unknowns.throttle                          = 0.8 * ones_row(1)  
            #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.1)  
            #mission.append_segment(segment)  
             
        
            ## ------------------------------------------------------------------
            ##  Downleg_Altitude Segment Flight 1 
            ## ------------------------------------------------------------------ 
            #segment = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment) 
            #segment.tag = 'Downleg' + "_F_" + str(flight_no) + "_D" + str (day)
            #segment.analyses.extend(analyses.base) 
            #segment.air_speed_start                                  = 110 * Units['mph']   
            #segment.air_speed_end                                    = 45.0 * Units['m/s']            
            #segment.distance                                         =  6000 * Units.feet
            #segment.acceleration                                     = -0.05 * Units['m/s/s']   
            #segment.state.unknowns.throttle                          = 0.5 * ones_row(1)  
            #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.1)  
            #mission.append_segment(segment)       
            
            #if reserve_segment:
                ## ------------------------------------------------------------------
                ##  Reserve Climb 
                ## ------------------------------------------------------------------ 
                #segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
                #segment.tag = 'Reserve_Climb'   + "_F_" + str(flight_no) + "_D" + str (day)       
                #segment.analyses.extend( analyses.base )       
                #segment.altitude_start                                   = 1000.0 * Units.feet
                #segment.altitude_end                                     = 1500 * Units.feet
                #segment.air_speed_start                                  = 45.0 * Units['m/s']         
                #segment.air_speed_end                                    = 150.* Units['mph']   
                #segment.climb_rate                                       = 500* Units['ft/min']  
                #segment.state.unknowns.throttle                          = 0.85 * ones_row(1)  
                #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.3)  
                #mission.append_segment(segment)   
                
                ## ------------------------------------------------------------------
                ##  Researve Cruise Segment 
                ## ------------------------------------------------------------------ 
                #segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment) 
                #segment.tag = 'Reserve_Cruise'  + "_F_" + str(flight_no) + "_D" + str (day) 
                #segment.analyses.extend(analyses.base) 
                #segment.air_speed                                        = 150.* Units['mph']   
                #segment.distance                                         = cruise_distance*0.1 
                #segment.state.unknowns.throttle                          = 0.8 * ones_row(1)  
                #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.12)  
                #mission.append_segment(segment)   
              
                ## ------------------------------------------------------------------
                ##  Researve Descent
                ## ------------------------------------------------------------------ 
                #segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
                #segment.tag = 'Reserve_Descent' + "_F_" + str(flight_no) + "_D" + str (day)
                #segment.analyses.extend( analyses.base )         
                #segment.altitude_start                                   = 1500 * Units.feet
                #segment.altitude_end                                     = 1000.0 * Units.feet
                #segment.air_speed_start                                  = 150.* Units['mph']  
                #segment.air_speed_end                                    = Vstall*1.2  
                #segment.climb_rate                                       = -200 * Units['ft/min']  
                #segment.state.unknowns.throttle                          = 0.8 * ones_row(1)  
                #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.1)  
                #mission.append_segment(segment)   
             
            ## ------------------------------------------------------------------
            ##  Baseleg Segment Flight 1  
            ## ------------------------------------------------------------------ 
            #segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            #segment.tag = 'Baseleg' + "_F_" + str(flight_no) + "_D" + str (day)
            #segment.analyses.extend( analyses.base)   
            #segment.altitude_start                                   = 1000 * Units.feet
            #segment.altitude_end                                     = 500.0 * Units.feet
            #segment.air_speed_start                                  = Vstall*1.2  
            #segment.air_speed_end                                    = Vstall*1.1   
            #segment.climb_rate                                       = -300 * Units['ft/min']
            #segment.state.unknowns.throttle                          =  0.7 * ones_row(1)  
            #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.3  )  
            #mission.append_segment(segment)   
        
            ## ------------------------------------------------------------------
            ##  Final Approach Segment Flight 1  
            ## ------------------------------------------------------------------ 
            #segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            #segment_name = 'Final_Approach' + "_F_" + str(flight_no) + "_D" + str (day)
            #segment.tag = segment_name          
            #segment.analyses.extend( analyses.base)         
            #segment.altitude_start                                   = 500.0 * Units.feet
            #segment.altitude_end                                     = 0.0 * Units.feet
            #segment.air_speed_start                                  = Vstall*1.1  
            #segment.air_speed_end                                    = Vstall    
            #segment.climb_rate                                       = -300 * Units['ft/min']          
            #segment.state.unknowns.throttle                          =  0.7 * ones_row(1) 
            #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.3  )  
            #mission.append_segment(segment)   
            
            #if recharge_battery:
                ## ------------------------------------------------------------------
                ##  Charge Segment: 
                ## ------------------------------------------------------------------     
                ## Charge Model 
                #segment                                             = Segments.Ground.Battery_Charge_Discharge(base_segment)     
                #segment.tag  = 'Charge Day ' + "_F_" + str(flight_no) + "_D" + str (day)  
                #segment.analyses.extend(analyses.base)           
                #segment.battery_discharge                                = False     
                #if flight_no  == flights_per_day:  
                    #segment.increment_battery_cycle_day=True            
                #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)    
                #mission.append_segment(segment)   
    
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


def plot_results(results,line_style = 'bo-'):  
    
    
    plot_flight_conditions(results, line_style) 
    
    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style) 
    
    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results, line_style)  
    
    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results, line_style)
    
    # Plot Aircraft Electronics
    plot_battery_pack_conditions(results, line_style)
    plot_battery_cell_conditions(results, line_style)
    
    # Plot Propeller Conditions 
    plot_propeller_conditions(results, line_style) 
    
    # Plot Electric Motor and Propeller Efficiencies 
    plot_eMotor_Prop_efficiencies(results, line_style)
    
    # Plot propeller Disc and Power Loading
    plot_disc_power_loading(results, line_style)    
                        
    return 

def save_results(results):

    # Store data (serialize)
    with open('ECTOL_2P_Results.pkl', 'wb') as file:
        pickle.dump(results, file)
        
    return   

if __name__ == '__main__': 
    main()    
    plt.show()