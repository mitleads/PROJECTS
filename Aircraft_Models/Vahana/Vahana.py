# Vahana.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data 
import os
from copy import deepcopy
from SUAVE.Plots.Performance.Mission_Plots                         import *  
from SUAVE.Plots.Geometry                                          import *   
from SUAVE.Methods.Power.Battery.Sizing                            import initialize_from_circuit_configuration 
from SUAVE.Methods.Propulsion.electric_motor_sizing                import size_optimal_motor
from SUAVE.Methods.Weights.Correlations.Propulsion                 import nasa_motor
from SUAVE.Methods.Propulsion                                      import propeller_design 
from SUAVE.Methods.Weights.Buildups.eVTOL.empty                    import empty
from SUAVE.Methods.Weights.Buildups.eVTOL.converge_evtol_weight    import converge_evtol_weight 
from SUAVE.Methods.Center_of_Gravity.compute_component_centers_of_gravity import compute_component_centers_of_gravity
import time 
import pickle 
#import vsp 
#from SUAVE.Input_Output.OpenVSP.vsp_write import write

import numpy as np
import pylab as plt 


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main(): 
    ti = time.time()
    # build the vehicle, configs, and analyses
    configs, analyses = full_setup()
 
    breakdown = empty(configs.base)
    print(breakdown)    
    
    configs.finalize()
    analyses.finalize()    

    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate() 

    tf = time.time()
    print ('time taken: ' + str(round(((tf-ti)/60),3)) + ' mins')       
    
    # save results  
    save_results(results)
    
    # plot the results
    plot_results(results) 

    return

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------
def full_setup():    

    # vehicle data
    vehicle  = vehicle_setup()
    #write(vehicle, "Tiltwing_VSP") 
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
    #  Noise Analysis
    noise = SUAVE.Analyses.Noise.Fidelity_One()   
    noise.geometry = vehicle
    noise.settings.level_ground_microphone_x_resolution = 10
    noise.settings.level_ground_microphone_y_resolution = 5
    noise.settings.level_ground_microphone_min_y        = 1E-6
    noise.settings.level_ground_microphone_max_y        = 2500
    noise.settings.level_ground_microphone_min_x        = -100
    noise.settings.level_ground_microphone_max_x        = 100
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

    return analyses    


def vehicle_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------     
    vehicle                                     = SUAVE.Vehicle()
    vehicle.tag                                 = 'Vahana'
    vehicle.configuration                       = 'eVTOL'
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.takeoff             = 735. 
    vehicle.mass_properties.operating_empty     = 735.
    vehicle.mass_properties.max_takeoff         = 735.
    vehicle.mass_properties.center_of_gravity   = [[ 2.0144,   0.  ,  0.]]
 
    vehicle.passengers                          = 0
    vehicle.envelope.ultimate_load              = 5.7
    vehicle.envelope.limit_load                 = 3.     

    # ------------------------------------------------------    
    # WINGS    
    # ------------------------------------------------------  
    wing                                        = SUAVE.Components.Wings.Main_Wing()
    wing.tag                                    = 'canard_wing'  
    wing.aspect_ratio                           = 11.37706641  
    wing.sweeps.quarter_chord                   = 0.0
    wing.thickness_to_chord                     = 0.18  
    wing.taper                                  = 1.  
    wing.spans.projected                        = 6.65 
    wing.chords.root                            = 0.95 
    wing.total_length                           = 0.95   
    wing.chords.tip                             = 0.95 
    wing.chords.mean_aerodynamic                = 0.95   
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = wing.chords.root*wing.spans.projected 
    wing.areas.wetted                           = 2*wing.chords.root*wing.spans.projected*0.95  
    wing.areas.exposed                          = 2*wing.chords.root*wing.spans.projected*0.95 
    wing.twists.root                            = 0.  
    wing.twists.tip                             = 0.  
    wing.origin                                 = [[0.1,  0.0 , 0.0]]  
    wing.aerodynamic_center                     = [0., 0., 0.]     
    wing.winglet_fraction                       = 0.0  
    wing.symmetric                              = True        
                                                
    # add to vehicle                                          
    vehicle.append_component(wing)                            
                                                
    wing                                        = SUAVE.Components.Wings.Main_Wing()
    wing.tag                                    = 'main_wing'  
    wing.aspect_ratio                           = 11.37706641  
    wing.sweeps.quarter_chord                   = 0.0
    wing.thickness_to_chord                     = 0.18  
    wing.taper                                  = 1.  
    wing.spans.projected                        = 6.65 
    wing.chords.root                            = 0.95 
    wing.total_length                           = 0.95   
    wing.chords.tip                             = 0.95 
    wing.chords.mean_aerodynamic                = 0.95   
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = wing.chords.root*wing.spans.projected 
    wing.areas.wetted                           = 2*wing.chords.root*wing.spans.projected*0.95  
    wing.areas.exposed                          = 2*wing.chords.root*wing.spans.projected*0.95 
    wing.twists.root                            = 0.  
    wing.twists.tip                             = 0.  
    wing.origin                                 = [[ 5.138, 0.0  ,  1.323 ]]  # for images 1.54
    wing.aerodynamic_center                     = [0., 0., 0.]     
    wing.winglet_fraction                       = 0.0  
    wing.symmetric                              = True 

    vehicle.reference_area                      = 2*wing.areas.reference 

    # add to vehicle 
    vehicle.append_component(wing)   


    # ------------------------------------------------------    
    # FUSELAGE    
    # ------------------------------------------------------    
    # FUSELAGE PROPERTIES                       
    fuselage                                    = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 0.  
    fuselage.seat_pitch                         = 1.  
    fuselage.fineness.nose                      = 1.5 
    fuselage.fineness.tail                      = 4.0 
    fuselage.lengths.nose                       = 1.7   
    fuselage.lengths.tail                       = 2.7 
    fuselage.lengths.cabin                      = 1.7  
    fuselage.lengths.total                      = 6.1  
    fuselage.width                              = 1.15  
    fuselage.heights.maximum                    = 1.7 
    fuselage.heights.at_quarter_length          = 1.2  
    fuselage.heights.at_wing_root_quarter_chord = 1.7  
    fuselage.heights.at_three_quarters_length   = 0.75 
    fuselage.areas.wetted                       = 12.97989862  
    fuselage.areas.front_projected              = 1.365211404  
    fuselage.effective_diameter                 = 1.318423736  
    fuselage.differential_pressure              = 0.  

    # Segment  
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment() 
    segment.tag                                 = 'segment_0'   
    segment.percent_x_location                  = 0.  
    segment.percent_z_location                  = 0.  
    segment.height                              = 0.09  
    segment.width                               = 0.23473  
    segment.length                              = 0.  
    segment.effective_diameter                  = 0. 
    fuselage.Segments.append(segment)             

    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_1'   
    segment.percent_x_location                  = 0.97675/6.1 
    segment.percent_z_location                  = 0.21977/6.1
    segment.height                              = 0.9027  
    segment.width                               = 1.01709  
    fuselage.Segments.append(segment)             


    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_2'    
    segment.percent_x_location                  = 1.93556/6.1 
    segment.percent_z_location                  = 0.39371/6.1
    segment.height                              = 1.30558   
    segment.width                               = 1.38871  
    fuselage.Segments.append(segment)             


    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_3'    
    segment.percent_x_location                  = 3.44137/6.1 
    segment.percent_z_location                  = 0.57143/6.1
    segment.height                              = 1.52588 
    segment.width                               = 1.47074 
    fuselage.Segments.append(segment)             

    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 4.61031/6.1
    segment.percent_z_location                  = 0.10893
    segment.height                              = 1.3906
    segment.width                               = 1.11463  
    fuselage.Segments.append(segment)              

    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.9827
    segment.percent_z_location                  = 0.180
    segment.height                              = 0.6145
    segment.width                               = 0.3838
    fuselage.Segments.append(segment)            
    
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 1. 
    segment.percent_z_location                  = 0.2058
    segment.height                              = 0.4
    segment.width                               = 0.25
    fuselage.Segments.append(segment)        

    # add to vehicle
    vehicle.append_component(fuselage)    


    #------------------------------------------------------------------
    # PROPULSOR
    #------------------------------------------------------------------
    net                            = SUAVE.Components.Energy.Networks.Battery_Propeller()
    net.number_of_propeller_engines= 8
    net.thrust_angle               = 0.0   * Units.degrees #  conversion to radians, 
    net.nacelle_diameter           = 0.4   # https://www.magicall.biz/products/integrated-motor-controller-magidrive/
    net.engine_length              = 0.95 
    net.areas                      = Data()
    net.areas.wetted               = np.pi*net.nacelle_diameter*net.engine_length + 0.5*np.pi*net.nacelle_diameter**2    
    net.identical_lift_rotors      = True  

        # Component 1 the ESC
    esc                            = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency                 = 0.95
    net.esc                        = esc 

    # Component 2 the Payload
    payload                        = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw             = 10. # Watts 
    payload.mass_properties.mass   = 0.0 * Units.kg
    net.payload                    = payload

    # Component 3 the Avionics    
    avionics                       = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw            = 20. # Watts  
    net.avionics                   = avionics   

    # Component 4 Miscellaneous Systems 
    sys                            = SUAVE.Components.Systems.System()
    sys.mass_properties.mass       = 5 # kg      

    # Component 5 the Battery      
    bat                         = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650()   
    bat.cell.surface_area       = (np.pi*bat.cell.height*bat.cell.diameter)  
    bat.pack_config.series      =  80  
    bat.pack_config.parallel    =  40  
    initialize_from_circuit_configuration(bat)    
    net.voltage                 = bat.max_voltage

    # Here we, are going to assume a battery pack module shape. This step is optional but
    # required for thermal analysis of the pack. We will assume that all cells electrically connected 
    # in series wihtin the module are arranged in one row normal direction to the airflow. Likewise ,
    # all cells electrically in paralllel are arranged in the direction to the cooling fluid  

    number_of_modules                = 10
    bat.module_config.total          = int(np.ceil(bat.pack_config.total/number_of_modules))
    bat.module_config.normal_count   = int(np.ceil(bat.module_config.total/bat.pack_config.series))
    bat.module_config.parallel_count = int(np.ceil(bat.module_config.total/bat.pack_config.parallel))
    net.battery                      = bat  


    # Component 6 the Rotor 
    # Design Rotors
    #------------------------------------------------------------------
    # atmosphere conditions
    speed_of_sound                 = 340 
    Lift                           = vehicle.mass_properties.takeoff*9.81

    # Create propeller geometry
    prop                          = SUAVE.Components.Energy.Converters.Propeller()  
    prop.tip_radius               = 0.8875  
    prop.hub_radius               = 0.15 * prop.tip_radius
    prop.disc_area                = np.pi*(prop.tip_radius**2)   
    prop.design_tip_mach          = 0.6
    prop.number_of_blades         = 3  
    prop.orientation_euler_angles     = [0.,0.,0.]
    prop.freestream_velocity      = 30     
    prop.angular_velocity         = prop.design_tip_mach*speed_of_sound/prop.tip_radius      
    prop.design_Cl                = 0.7
    prop.design_altitude          = 500 * Units.feet        
    prop.number_azimuthal_stations = 12 # reducing matrix for noise 
    prop.design_thrust            = Lift/(net.number_of_propeller_engines-1) # contingency for one-engine-inoperative condition 
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    prop.airfoil_geometry         =  [ rel_path + '../Airfoils/NACA_4412.txt']
    prop.airfoil_polars           = [[ rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                        rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                      rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                      rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                      rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]  
    prop.airfoil_polar_stations   = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    prop                          = propeller_design(prop)   
    prop.variable_pitch           = True 
    rotation                       = [1,-1,1,-1,1,-1,1,-1]

    # Front Rotors Locations 
    origins = [[-0.2, 1.347, 0.0], [-0.2, 3.2969999999999997, 0.0], [-0.2, -1.347, 0.0], [-0.2, -3.2969999999999997, 0.0],\
               [4.938, 1.347, 1.54], [4.938, 3.2969999999999997, 1.54],[4.938, -1.347, 1.54], [4.938, -3.2969999999999997, 1.54]] 
    
    for ii in range(8):
        lift_rotor          = deepcopy(prop)
        lift_rotor.tag      = 'rotor_' + str(ii+1)
        lift_rotor.origin   = [origins[ii]]
        lift_rotor.rotation = rotation[ii]
        net.propellers.append(lift_rotor) 


    # Motor
    #------------------------------------------------------------------
    # Design Motors
    #------------------------------------------------------------------
    motor                           = SUAVE.Components.Energy.Converters.Motor() 
    motor.origin                    = prop.origin  
    motor.efficiency                = 0.95 
    motor.nominal_voltage           = bat.max_voltage * 0.75
    motor.propeller_radius          = prop.tip_radius 
    motor.no_load_current           = 0.01
    motor                           = size_optimal_motor(motor,prop) 
    motor.mass_properties.mass      = nasa_motor(motor.design_torque)  

    for ii in range(8):
        rotor_motor = deepcopy(motor)
        rotor_motor.tag    = 'motor_' + str(ii+1)
        rotor_motor.origin = [origins[ii]]
        net.propeller_motors.append(rotor_motor)  
   
    # Add extra drag sources from motors, props, and landing gear. All of these hand measured 
    motor_height                     = .25 * Units.feet
    motor_width                      =  1.6 * Units.feet 
    propeller_width                  = 1. * Units.inches
    propeller_height                 = propeller_width *.12 
    main_gear_width                  = 1.5 * Units.inches
    main_gear_length                 = 2.5 * Units.feet 
    nose_gear_width                  = 2. * Units.inches
    nose_gear_length                 = 2. * Units.feet 
    nose_tire_height                 = (0.7 + 0.4) * Units.feet
    nose_tire_width                  = 0.4 * Units.feet  
    main_tire_height                 = (0.75 + 0.5) * Units.feet
    main_tire_width                  = 4. * Units.inches 
    total_excrescence_area_spin      = 12.*motor_height*motor_width + 2.* main_gear_length*main_gear_width \
        + nose_gear_width*nose_gear_length + 2 * main_tire_height*main_tire_width\
                                       + nose_tire_height*nose_tire_width 
    total_excrescence_area_no_spin   = total_excrescence_area_spin + 12*propeller_height*propeller_width  
    vehicle.excrescence_area_no_spin = total_excrescence_area_no_spin 
    vehicle.excrescence_area_spin    = total_excrescence_area_spin  
    
    # append motor origin spanwise locations onto wing data structure
    motor_origins_front                                   = np.array(origins[:4])
    motor_origins_rear                                    = np.array(origins[5:])
    vehicle.wings['canard_wing'].motor_spanwise_locations = motor_origins_front[:,1]/ vehicle.wings['canard_wing'].spans.projected
    vehicle.wings['canard_wing'].motor_spanwise_locations = motor_origins_front[:,1]/ vehicle.wings['canard_wing'].spans.projected
    vehicle.wings['main_wing'].motor_spanwise_locations   = motor_origins_rear[:,1]/ vehicle.wings['main_wing'].spans.projected
    
    vehicle.append_component(net)
    
    converge_evtol_weight(vehicle) 
    breakdown  = empty(vehicle)
    print(breakdown)    
    vehicle.weight_breakdown 
    compute_component_centers_of_gravity(vehicle)
    vehicle.center_of_gravity()    
    
    
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------ 
    # plot vehicle 
    #plot_vehicle(vehicle,plot_control_points = False) 
    #plt.show()  

    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    '''
    The configration set up below the scheduling of the nacelle angle and vehicle speed.
    Since one propeller operates at varying flight conditions, one must perscribe  the 
    pitch command of the propeller which us used in the variable pitch model in the analyses
    Note: low pitch at take off & low speeds, high pitch at cruise
    '''
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    configs                                     = SUAVE.Components.Configs.Config.Container() 
    base_config                                 = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag                             = 'base'
    configs.append(base_config)
 
    # ------------------------------------------------------------------
    #   Hover Climb Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'vertical_climb'
    vector_angle                                      = 90.0 * Units.degrees
    config.networks.battery_propeller.thrust_angle    = vector_angle
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle   
    config.networks.battery_propeller.pitch_command   = -5.  * Units.degrees   
    configs.append(config)

    # ------------------------------------------------------------------
    #    
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 45.0  * Units.degrees 
    config.tag                                        = 'vertical_transition'
    config.networks.battery_propeller.thrust_angle    = vector_angle
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle
    config.networks.battery_propeller.pitch_command   = -3.  * Units.degrees  # tried 3,0,-3
    configs.append(config)
    

    # ------------------------------------------------------------------
    #   Hover-to-Cruise Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'climb_transition'
    vector_angle                                      = 15.0  * Units.degrees  
    config.networks.battery_propeller.thrust_angle    = vector_angle
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    config.networks.battery_propeller.pitch_command   = 5.  * Units.degrees     
    configs.append(config)
     
    # ------------------------------------------------------------------
    #   Climb Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'climb'   
    vector_angle                                      = 0.0 * Units.degrees
    config.networks.battery_propeller.thrust_angle    = vector_angle
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle  
    config.networks.battery_propeller.pitch_command   = 18.  * Units.degrees   # 20 
    configs.append(config)    

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'cruise'   
    vector_angle                                      = 0.0 * Units.degrees
    config.networks.battery_propeller.thrust_angle    = vector_angle
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle  
    config.networks.battery_propeller.pitch_command   = 20.  * Units.degrees  
    configs.append(config)    
    
  

    # ------------------------------------------------------------------
    #   Approach Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'approach'   
    vector_angle                                      = 15.0 * Units.degrees
    config.networks.battery_propeller.thrust_angle    = vector_angle
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle  
    config.networks.battery_propeller.pitch_command   = 15.  * Units.degrees   # 20 
    configs.append(config)     
    
    # ------------------------------------------------------------------
    #   
    # ------------------------------------------------------------------ 
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 25.0  * Units.degrees   
    config.tag                                        = 'descent_transition'  
    config.networks.battery_propeller.thrust_angle    = vector_angle
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle
    config.networks.battery_propeller.pitch_command   = 5.  * Units.degrees   
    configs.append(config) 
    

    # ------------------------------------------------------------------
    #   Hover Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'vertical_descent'
    vector_angle                                      = 90.0  * Units.degrees  
    config.networks.battery_propeller.thrust_angle    = vector_angle
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle     
    config.networks.battery_propeller.pitch_command   = -5.  * Units.degrees  
    configs.append(config)

    return configs 


def mission_setup(analyses,vehicle ):
        
  
    starting_elevation = 0*Units.feet
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission     = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport            = SUAVE.Attributes.Airports.Airport() 
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments
    

    # base segment
    base_segment                                             = Segments.Segment() 
    base_segment.state.numerics.number_control_points        = 4
    ones_row                                                 = base_segment.state.ones_row
    base_segment.battery_discharge                           = True  
    base_segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
    base_segment.process.finalize.post_process.stability     = SUAVE.Methods.skip    
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  
    
 
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                                            = Segments.Hover.Hover(base_segment)
    segment.tag                                        = "Hover"   
    segment.analyses.extend(analyses.vertical_climb)  
    segment.altitude                                   = 500*Units.feet 
    segment.time                                       = 10 
    segment.battery_energy                             = vehicle.networks.battery_propeller.battery.max_energy   
    segment.state.unknowns.throttle                    = 0.9  * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06) 
    mission.append_segment(segment)  
 
  
 
    ## ------------------------------------------------------------------
    ##   First Climb Segment: Constant Speed, Constant Rate
    ## ------------------------------------------------------------------ 
    #segment                                            = Segments.Hover.Climb(base_segment)
    #segment.tag                                        = "Vertical_Climb"   
    #segment.analyses.extend(analyses.vertical_climb) 
    #segment.altitude_start                             = 0.0  * Units.ft + starting_elevation 
    #segment.altitude_end                               = 40.  * Units.ft + starting_elevation 
    #segment.climb_rate                                 = 300. * Units['ft/min']
    #segment.battery_energy                             = vehicle.networks.battery_propeller.battery.max_energy   
    #segment.state.unknowns.throttle                    = 0.9  * ones_row(1)  
    #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06) 
    #mission.append_segment(segment)  
    
    
    ## ------------------------------------------------------------------
    ##  First Transition Segment
    ## ------------------------------------------------------------------ 
    #segment                       = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    #segment.tag                   = "Vertical_Transition"  
    #segment.analyses.extend( analyses.vertical_transition) 
    #segment.altitude              = 40.  * Units.ft + starting_elevation 
    #segment.air_speed_start       = 300. * Units['ft/min']     
    #segment.air_speed_end         = 35 * Units['mph']     
    #segment.acceleration          = 9.81/5
    #segment.pitch_initial         = 1. * Units.degrees
    #segment.pitch_final           = 2. * Units.degrees 
    #segment.state.unknowns.throttle   = 0.8 * ones_row(1) 
    #segment                       = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.06) 
    #mission.append_segment(segment)


    ## ------------------------------------------------------------------
    ##   First Cruise Segment: Constant Acceleration, Constant Altitude
    ## ------------------------------------------------------------------ 
    #segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    #segment.tag                      = "Climb_Transition_1" 
    #segment.analyses.extend(analyses.climb_transition) 
    #segment.climb_rate               = 500. * Units['ft/min']
    #segment.air_speed_start          = 35.   * Units['mph']
    #segment.air_speed_end            = 85.   * Units['mph'] 
    #segment.altitude_start           = 40.0 * Units.ft   + starting_elevation 
    #segment.altitude_end             = 100.0 * Units.ft  + starting_elevation 
    #segment.state.unknowns.throttle   = 0.80 * ones_row(1) 
    #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.06)       
    #mission.append_segment(segment)   
    
     
    ## ------------------------------------------------------------------
    ##  Second Transition Segment
    ## ------------------------------------------------------------------ 
    #segment                           = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    #segment.tag                       = "Climb_Transition_2"  
    #segment.analyses.extend( analyses.climb_transition) 
    #segment.altitude                  = 100.  * Units.ft + starting_elevation 
    #segment.air_speed_start           = 85.  * Units['mph'] 
    #segment.air_speed_end             = 125.  * Units['mph']  
    #segment.acceleration              = 9.81/5
    #segment.pitch_initial             = 2. * Units.degrees
    #segment.pitch_final               = 5. * Units.degrees
    #segment.state.unknowns.throttle   = 0.80 * ones_row(1) 
    #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.06)     
    #mission.append_segment(segment)

    ## ------------------------------------------------------------------
    ##   First Cruise Segment: Constant Acceleration, Constant Altitude
    ## ------------------------------------------------------------------ 
    #segment                           = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    #segment.tag                       = "Climb"  
    #segment.analyses.extend(analyses.climb) 
    #segment.climb_rate                = 500. * Units['ft/min']
    #segment.air_speed_start           = 125.   * Units['mph']
    #segment.air_speed_end             = 175.   * Units['mph'] 
    #segment.altitude_start            = 100.0 * Units.ft  + starting_elevation 
    #segment.altitude_end              = 2500.0 * Units.ft                
    #segment.state.unknowns.throttle   = 0.80 * ones_row(1) 
    #segment.battery_energy                             = vehicle.networks.battery_propeller.battery.max_energy  
    #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)     
    #mission.append_segment(segment)     

    ## ------------------------------------------------------------------
    ##   First Cruise Segment: Constant Acceleration, Constant Altitude
    ## ------------------------------------------------------------------ 
    #segment                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    #segment.tag                      = "Cruise"  
    #segment.analyses.extend(analyses.cruise) 
    #segment.altitude                 = 2500.0 * Units.ft
    #segment.air_speed                = 175.   * Units['mph']  
    #segment.distance                 = 10*Units.nmi
    #segment.state.unknowns.throttle  = 0.8 * ones_row(1) 
    #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment )     
    #mission.append_segment(segment)     
    
    ## ------------------------------------------------------------------
    ##    Descent Segment: Constant Acceleration, Constant Altitude
    ## ------------------------------------------------------------------ 
    #segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    #segment.tag                      = "Descent"  
    #segment.analyses.extend(analyses.climb)
    #segment.climb_rate               = -300. * Units['ft/min']
    #segment.air_speed_start          = 175.   * Units['mph']
    #segment.air_speed_end            = 100.   * Units['mph'] 
    #segment.altitude_start           = 2500.0 * Units.ft
    #segment.altitude_end             = 100.0 * Units.ft + starting_elevation      
    #segment.state.unknowns.throttle  = 0.6 * ones_row(1) 
    #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
    #mission.append_segment(segment)     

    ## ------------------------------------------------------------------
    ##   Reserve Climb Segment 
    ## ------------------------------------------------------------------ 
    #segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    #segment.tag                      = "Reserve_Climb"   
    #segment.analyses.extend(analyses.climb) 
    #segment.climb_rate               = 500. * Units['ft/min']
    #segment.air_speed_start          = 100.   * Units['mph'] 
    #segment.air_speed_end            = 150.   * Units['mph'] 
    #segment.altitude_start           = 100.0 * Units.ft+ starting_elevation 
    #segment.altitude_end             = 1000.0 * Units.ft              
    #segment.state.unknowns.throttle  = 0.8 * ones_row(1) 
    #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
    #mission.append_segment(segment)      
 
    ## ------------------------------------------------------------------
    ##   First Cruise Segment: Constant Acceleration, Constant Altitude
    ## ------------------------------------------------------------------ 
    #segment                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment) 
    #segment.tag                      = "Reserve_Cruise"  
    #segment.analyses.extend(analyses.cruise)  
    #segment.air_speed                = 150.   * Units['mph'] 
    #segment.distance                 = cruise_distance*0.1  - 4.*Units.nmi     
    #segment.state.unknowns.throttle  = 0.8 * ones_row(1) 
    #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.01)  
    #mission.append_segment(segment)     
 
    ## ------------------------------------------------------------------
    ##   Reserve Descent Segment: Constant Acceleration, Constant Altitude
    ## ------------------------------------------------------------------ 
    #segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    #segment.tag                      = "Reserve_Descent" 
    #segment.analyses.extend(analyses.climb)
    #segment.climb_rate               = -300. * Units['ft/min']
    #segment.air_speed_start          = 150.   * Units['mph']
    #segment.air_speed_end            = 100.   * Units['mph']
    #segment.altitude_start           = 1000.0 * Units.ft
    #segment.altitude_end             = 100.0 * Units.ft    + starting_elevation                
    #segment.state.unknowns.throttle  = 0.6 * ones_row(1) 
    #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
    #mission.append_segment(segment)        
    
    ## ------------------------------------------------------------------
    ##  Forth Transition Segment
    ## ------------------------------------------------------------------ 
    #segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    #segment.tag                      = "Approach_Transition"   
    #segment.analyses.extend( analyses.approach)  
    #segment.climb_rate               = -200. * Units['ft/min']
    #segment.air_speed_start          = 100.   * Units['mph'] 
    #segment.air_speed_end            = 55.   * Units['mph'] 
    #segment.altitude_start           = 100.0 * Units.ft     + starting_elevation
    #segment.altitude_end             = 40.0 * Units.ft     + starting_elevation              
    #segment.state.unknowns.throttle  = 0.6 * ones_row(1) 
    #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
    #mission.append_segment(segment)     
    
    ## ------------------------------------------------------------------
    ##  Forth Transition Segment
    ## ------------------------------------------------------------------ 
    #segment                          = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)  
    #segment.tag                      = "Descent_Transition" 
    #segment.analyses.extend( analyses.descent_transition)   
    #segment.altitude                 = 40.  * Units.ft+ starting_elevation 
    #segment.air_speed_start          = 55 * Units['mph']    
    #segment.air_speed_end            = 300. * Units['ft/min'] 
    #segment.acceleration             = -0.5 * Units['m/s/s']   
    #segment.pitch_initial            = 1. * Units.degrees
    #segment.pitch_final              = 2. * Units.degrees               
    #segment.state.unknowns.throttle  = 0.6 * ones_row(1) 
    #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment , initial_power_coefficient = 0.01) 
    #mission.append_segment(segment)     

    ## ------------------------------------------------------------------
    ##   Descent Segment: Constant Speed, Constant Rate
    ## ------------------------------------------------------------------ 
    #segment                          = Segments.Hover.Descent(base_segment)
    #segment.tag                      = "Vertical_Descent"  
    #segment.analyses.extend( analyses.vertical_descent) 
    #segment.altitude_start           = 40.0  * Units.ft + starting_elevation 
    #segment.altitude_end             = 0.  * Units.ft + starting_elevation 
    #segment.descent_rate             = 300. * Units['ft/min']  
    #segment.state.unknowns.throttle  = 0.6  * ones_row(1)  
    #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06)  
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


# ----------------------------------------------------------------------
#   Plot Results
# ----------------------------------------------------------------------

def plot_results(results,line_style = 'bo-'):  

    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style) 

    # Plot Aerodynamic  Forces
    plot_aerodynamic_forces(results)

    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results, line_style)

    # Plot Aircraft Electronics
    plot_battery_pack_conditions(results, line_style)

    # Plot Propeller Conditions 
    plot_propeller_conditions(results, line_style) 

    ## Plot Electric Motor and Propeller Efficiencies 
    #plot_eMotor_Prop_efficiencies(results, line_style)

    ## Plot propeller Disc and Power Loading
    #plot_disc_power_loading(results, line_style)   


    return 

def save_results(results):

    # Store data (serialize)
    with open('Tiltwing.pkl', 'wb') as file:
        pickle.dump(results, file)

    return  

if __name__ == '__main__': 
    main()    
    plt.show()
