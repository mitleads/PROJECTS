# Tiltwing_Vehicle.py 

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data

from SUAVE.Methods.Power.Battery.Sizing                                      import initialize_from_mass 
from SUAVE.Methods.Power.Battery.Sizing                                      import initialize_from_circuit_configuration 
from SUAVE.Methods.Propulsion.electric_motor_sizing                          import size_optimal_motor
from SUAVE.Methods.Weights.Correlations.Propulsion                           import nasa_motor
from SUAVE.Methods.Propulsion                                                import propeller_design
from SUAVE.Plots.Geometry                                                    import *
from SUAVE.Plots.Performance.Mission_Plots                                   import *
from SUAVE.Methods.Weights.Buildups.eVTOL.empty                              import empty
from SUAVE.Methods.Center_of_Gravity.compute_component_centers_of_gravity    import compute_component_centers_of_gravity
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools.generate_microphone_points import generate_building_microphone_points
from SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform           import wing_planform 
#from SUAVE.Methods.Weights.Buildups.eVTOL.converge_evtol_weight              import converge_evtol_weight  

from copy import deepcopy
import numpy as np
import os
import pylab as plt 
import pickle 
import time  

try:
    import vsp 
    from SUAVE.Input_Output.OpenVSP.vsp_write import write 
except ImportError:
    # This allows SUAVE to build without OpenVSP
    pass

# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def vehicle_setup(MTOW=None): 
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------     
    vehicle                                     = SUAVE.Vehicle()
    vehicle.tag                                 = 'Tiltwing_CRM'
    vehicle.configuration                       = 'eVTOL'
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    if MTOW == None:
        MTOW = 2900 
    vehicle.mass_properties.max_takeoff         = MTOW
    vehicle.mass_properties.takeoff             = vehicle.mass_properties.max_takeoff
    vehicle.mass_properties.operating_empty     = vehicle.mass_properties.max_takeoff
    vehicle.mass_properties.center_of_gravity   = [[ 2.0144,   0.  ,  0.]] 
    vehicle.passengers                          = 6
    vehicle.envelope.ultimate_load              = 5.7
    vehicle.envelope.limit_load                 = 3.     
    
    # ------------------------------------------------------    
    # WINGS    
    # ------------------------------------------------------  
    wing                                        = SUAVE.Components.Wings.Main_Wing()
    wing.tag                                    = 'canard_wing'  
    wing.aspect_ratio                           = 8.51507 
    wing.sweeps.quarter_chord                   = 0.0
    wing.thickness_to_chord                     = 0.16 
    wing.taper                                  = 1.  
    span                                        = 9.2
    wing.spans.projected                        = span 
    chord                                       = 1.05 #0.85
    wing.chords.root                            = chord
    wing.total_length                           = chord
    wing.chords.tip                             = chord
    wing.chords.mean_aerodynamic                = chord
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = wing.chords.root*wing.spans.projected 
    wing.areas.wetted                           = 2*wing.chords.root*wing.spans.projected*0.95  
    wing.areas.exposed                          = 2*wing.chords.root*wing.spans.projected*0.95 
    wing.twists.root                            = 0.* Units.degrees  
    wing.twists.tip                             = 0.  
    wing.origin                                 = [[0.1,  0.0 , 0.0]]  
    wing.aerodynamic_center                     = [0., 0., 0.]     
    wing.winglet_fraction                       = 0.0  
    wing.symmetric                              = True        
                                                
    # add to vehicle   
    vehicle.append_component(wing)                            
                                                
    wing                                        = SUAVE.Components.Wings.Main_Wing()
    wing.tag                                    = 'main_wing'  
    wing.aspect_ratio                           = 8.51507 
    wing.sweeps.quarter_chord                   = 0.0
    wing.thickness_to_chord                     = 0.16 
    wing.taper                                  = 1.  
    wing.spans.projected                        = span 
    wing.chords.root                            = chord
    wing.total_length                           = chord 
    wing.chords.tip                             = chord
    wing.chords.mean_aerodynamic                = chord
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = wing.chords.root*wing.spans.projected 
    wing.areas.wetted                           = 2*wing.chords.root*wing.spans.projected*0.95  
    wing.areas.exposed                          = 2*wing.chords.root*wing.spans.projected*0.95 
    wing.twists.root                            = 0. * Units.degrees 
    wing.twists.tip                             = 0.   
    wing.origin                                 = [[ 5.138, 0.0  ,  1.323 ]]  # for images 1.54
    wing.aerodynamic_center                     = [0., 0., 0.]     
    wing.winglet_fraction                       = 0.0  
    wing.symmetric                              = True 

    # compute reference properties 
    wing_planform(wing) 
    wing = wing_planform(wing)
    vehicle.reference_area = wing.areas.reference*2   
     
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
    fuselage.lengths.total                      = 6.3  
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
    segment.percent_z_location                  = 0.81577/6.1
    segment.height                              = 1.14788 
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
    net                              = SUAVE.Components.Energy.Networks.Battery_Propeller()
    net.number_of_propeller_engines  = 8
    net.thrust_angle                 = 0.0   * Units.degrees #  conversion to radians,  
    net.engine_length                = 0.95 
    net.areas                        = Data()
    net.identical_propellers         = True  
    
    # Component 1 the ESC
    esc                            = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency                 = 0.95
    net.esc                        = esc 
    
    # Component 2 the Payload
    payload                        = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw             = 10. # Watts 
    payload.mass_properties.mass   = 1.0 * Units.kg
    net.payload                    = payload
    
    # Component 3 the Avionics    
    avionics                       = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw            = 20. # Watts  
    net.avionics                   = avionics   
    
    # Component 4 Miscellaneous Systems 
    sys                            = SUAVE.Components.Systems.System()
    sys.mass_properties.mass       = 5 # kg      
    
    # Component 5 the Battery       
    total_cells                          = 140*130   
    max_module_voltage                   = 50
    safety_factor                        = 1.5
     
    bat                                  = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650() 
    bat.pack_config.series               = 140  # CHANGE IN OPTIMIZER 
    bat.pack_config.parallel             = int(total_cells/bat.pack_config.series)
    initialize_from_circuit_configuration(bat)    
    net.voltage                          = bat.max_voltage  
    bat.module_config.number_of_modules  = 16 # CHANGE IN OPTIMIZER 
    bat.module_config.total              = int(np.ceil(bat.pack_config.total/bat.module_config.number_of_modules))
    bat.module_config.voltage            = net.voltage/bat.module_config.number_of_modules # must be less than max_module_voltage/safety_factor 
    bat.module_config.layout_ratio       = 0.5 # CHANGE IN OPTIMIZER 
    bat.module_config.normal_count       = int(bat.module_config.total**(bat.module_config.layout_ratio))
    bat.module_config.parallel_count     = int(bat.module_config.total**(1-bat.module_config.layout_ratio)) 
    net.battery                          = bat        
    
    # Component 6 the Rotor 
    # Design Rotors
    #------------------------------------------------------------------
    # atmosphere conditions
    speed_of_sound                 = 340
    rho                            = 1.22   
    Lift                           = vehicle.mass_properties.max_takeoff*9.81

    # Create propeller geometry
    prop_rotor                          = SUAVE.Components.Energy.Converters.Lift_Rotor()  
    prop_rotor.tip_radius               = 1.25  
    prop_rotor.hub_radius               = 0.15 * prop_rotor.tip_radius   
    prop_rotor.design_tip_mach          = 0.6 # gives better noise results and more realistic blade 
    prop_rotor.number_of_blades         = 3  
    prop_rotor.freestream_velocity      = 130 * Units.mph  # 10  
    prop_rotor.angular_velocity         = prop_rotor.design_tip_mach*speed_of_sound/prop_rotor.tip_radius      
    prop_rotor.design_Cl                = 0.7
    prop_rotor.design_altitude          = 500 * Units.feet                   
    prop_rotor.design_thrust            = Lift/(net.number_of_propeller_engines) # contingency for one-engine-inoperative condition and then turning off off-diagonal rotor
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    prop_rotor.airfoil_geometry         =  [ rel_path + '../Airfoils/NACA_4412.txt']
    prop_rotor.airfoil_polars           = [[ rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                       rel_path +  '../Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                       rel_path +  '../Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                       rel_path +  '../Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                       rel_path +  '../Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]  
    prop_rotor.airfoil_polar_stations   = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    prop_rotor                          = propeller_design(prop_rotor)   
    prop_rotor.variable_pitch           = True 
    prop_rotor.rotation                 = 1 

    # Rotors Locations  
    origins   = [[-0.3, 2.0, 0.0], [-0.3, 4.8, 0.0],[-0.3, -2.0, 0.0], [-0.3, -4.8, 0.0],\
               [4.7, 2.0 ,1.4], [4.7, 4.8, 1.4],[4.7, -2.0, 1.4], [4.7, -4.8, 1.4]]      

    for ii in range(8):
        tw_prop_rotor          = deepcopy(prop_rotor)
        tw_prop_rotor.tag      = 'prop_' + str(ii+1)
        tw_prop_rotor.origin   = [origins[ii]]
        net.propellers.append(tw_prop_rotor) 
        
    
    # Nacelles 
    nacelle                = SUAVE.Components.Nacelles.Nacelle()
    nacelle.tag            = 'prop_nacelle'
    nacelle.length         = 1.5
    nacelle.diameter       = 0.5
    nacelle.orientation_euler_angles  = [0.,0.,0.]    
    nacelle.flow_through   = False  
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_1'
    nac_segment.percent_x_location = 0.0  
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)    

    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_2'
    nac_segment.percent_x_location = 0.10  
    nac_segment.height             = 0.3
    nac_segment.width              = 0.3
    nacelle.append_segment(nac_segment)    
    
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_3'
    nac_segment.percent_x_location = 0.25  
    nac_segment.height             = 0.45
    nac_segment.width              = 0.45
    nacelle.append_segment(nac_segment)    
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_4'
    nac_segment.percent_x_location = 0.5 
    nac_segment.height             = 0.5
    nac_segment.width              = 0.5
    nacelle.append_segment(nac_segment)    

    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 0.75
    nac_segment.height             = 0.45
    nac_segment.width              = 0.45
    nacelle.append_segment(nac_segment)        

    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 0.9
    nac_segment.height             = 0.3
    nac_segment.width              = 0.3
    nacelle.append_segment(nac_segment)    
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_7'
    nac_segment.percent_x_location = 1.0  
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)     


    prop_nacelle_origins = [[-0.5, 2.0, 0.0], [-0.5, 4.8, 0.0],[-0.5, -2.0, 0.0], [-0.5, -4.8, 0.0],\
               [4.5, 2.0 ,1.4], [4.5, 4.8, 1.4],[4.5, -2.0, 1.4], [4.5, -4.8, 1.4]] 
    
    for ii in range(8):
        prop_nacelle          = deepcopy(nacelle)
        prop_nacelle.tag      = 'propeller_nacelle_' + str(ii+1) 
        prop_nacelle.origin   = [prop_nacelle_origins[ii]]
        vehicle.append_component(prop_nacelle)       
     
    
    # Motor
    #------------------------------------------------------------------
    # Design Motors
    #------------------------------------------------------------------
    motor                           = SUAVE.Components.Energy.Converters.Motor() 
    motor.origin                    = prop_rotor.origin  
    motor.efficiency                = 0.9  
    motor.nominal_voltage           = bat.max_voltage *0.8  
    motor.propeller_radius          = prop_rotor.tip_radius 
    motor.no_load_current           = 0.01  
    motor                           = size_optimal_motor(motor,prop_rotor) 
    motor.mass_properties.mass      = nasa_motor(motor.design_torque)  

    for ii in range(8):
        prop_motor = deepcopy(motor)
        prop_motor.tag    = 'motor_' + str(ii+1)
        prop_motor.origin = [origins[ii]]
        net.propeller_motors.append(prop_motor)  

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
    
    #converge_evtol_weight(vehicle,contingency_factor = 1.0 )
    

    breakdown = empty(vehicle,contingency_factor = 1.0 )
    print(breakdown)
    
    vehicle.weight_breakdown  =breakdown
    
    compute_component_centers_of_gravity(vehicle)
    vehicle.center_of_gravity()

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
    #   Hover Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'vertical_climb'
    vector_angle                                      = 90.0 * Units.degrees
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = -10.  * Units.degrees
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle
    configs.append(config) 

    # ------------------------------------------------------------------
    #  Vertical Transition 1 
    # ------------------------------------------------------------------  
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 35.0  * Units.degrees
    config.tag                                        = 'vertical_transition_1'
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = -5.  * Units.degrees
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)    


    # ------------------------------------------------------------------
    # Vertical Transition 2    
    # ------------------------------------------------------------------ 
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 25.0  * Units.degrees
    config.tag                                        = 'vertical_transition_2'
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = -5.  * Units.degrees
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)    
    

    # ------------------------------------------------------------------
    #   Hover-to-Cruise Configuration
    # ------------------------------------------------------------------ 
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 15.0  * Units.degrees
    config.tag                                        = 'climb_transition'
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = -3.  * Units.degrees
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)    

     
    # ------------------------------------------------------------------
    #   Climb Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'climb'
    vector_angle                                      = 0.0 * Units.degrees 
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = 0.  * Units.degrees 
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)
   

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------      
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'cruise'
    vector_angle                                      = 0.0 * Units.degrees 
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = 0.  * Units.degrees 
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)
  

    # ------------------------------------------------------------------
    #   Approach Configuration
    # ------------------------------------------------------------------  
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 15.0  * Units.degrees
    config.tag                                        = 'approach'  
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = 0.  * Units.degrees
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)    

    
    
    # ------------------------------------------------------------------
    #  Descent Transition
    # ------------------------------------------------------------------  
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 45.0  * Units.degrees
    config.tag                                        = 'descent_transition'  
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = -5.  * Units.degrees
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)    

    

    # ------------------------------------------------------------------
    #   Hover Configuration
    # ------------------------------------------------------------------ 
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'vertical_descent'
    vector_angle                                      = 90.0  * Units.degrees 
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = -10.  * Units.degrees 
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config) 

    return configs 
 