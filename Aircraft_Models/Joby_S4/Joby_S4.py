# TiltRotor.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data 
import os
import pickle
from SUAVE.Plots.Performance.Mission_Plots import *  
from SUAVE.Plots.Geometry  import * 
from SUAVE.Components.Energy.Networks.Lift_Cruise              import Lift_Cruise 
from SUAVE.Methods.Power.Battery.Sizing                        import initialize_from_mass
from SUAVE.Methods.Power.Battery.Sizing                        import initialize_from_circuit_configuration  
from SUAVE.Methods.Weights.Correlations.Propulsion             import nasa_motor
from SUAVE.Methods.Propulsion.electric_motor_sizing            import size_from_mass , size_optimal_motor
from SUAVE.Methods.Propulsion                                  import propeller_design , lift_rotor_design
#from SUAVE.Methods.Weights.Buildups.eVTOL.converge_evtol_weight    import converge_evtol_weight  
from SUAVE.Methods.Weights.Buildups.eVTOL.empty import empty

import vsp 
from SUAVE.Input_Output.OpenVSP.vsp_write import write

import numpy as np
import pylab as plt
from copy import deepcopy 


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():

    # build the vehicle, configs, and analyses
    configs, analyses = full_setup()


    breakdown = empty(configs.base)
    print(breakdown)    
    
    # configs.finalize()
    analyses.finalize()    

    # weight analysis
    #weights = analyses.weights
    #breakdown = weights.evaluate()  
    
    # mission analysis
    mission = analyses.missions.base
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
    write(vehicle, "Joby_S4") 
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
#   Build the Vehicle
# ----------------------------------------------------------------------
def vehicle_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    vehicle                                   = SUAVE.Vehicle()
    vehicle.tag                               = 'TiltRotor'
    vehicle.configuration                     = 'eVTOL'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.takeoff           = 2177
    vehicle.mass_properties.operating_empty   = 2177      
    vehicle.mass_properties.max_takeoff       = 2177             
    vehicle.mass_properties.center_of_gravity = [[2.0144,   0.  ,  0. ]]      
    vehicle.reference_area                    = 10.39
    vehicle.envelope.ultimate_load            = 5.7   
    vehicle.envelope.limit_load               = 3.  
    vehicle.passengers                        = 5

    # ------------------------------------------------------------------    
    # WINGS                                    
    # ------------------------------------------------------------------    
    # WING PROPERTIES           
    wing                                      = SUAVE.Components.Wings.Main_Wing()
    wing.tag                                  = 'main_wing'  
    wing.aspect_ratio                         = 9.11 
    wing.sweeps.quarter_chord                 = 0.0  
    wing.thickness_to_chord                   = 0.15
    wing.taper                                = 0.650 
    wing.spans.projected                      = 9.736 
    wing.chords.root                          = 1.57 
    wing.total_length                         = 1.57  
    wing.chords.tip                           = 0.66 
    wing.chords.mean_aerodynamic              = 1.069 
    wing.dihedral                             = 0   * Units.degrees  
    wing.areas.reference                      = 10.39 * 2  
    wing.areas.wetted                         = 10.39 * 2   
    wing.areas.exposed                        = 10.39 * 2   
    wing.twists.root                          = 0   * Units.degrees  
    wing.twists.tip                           = 0   * Units.degrees   
    wing.origin                               = [[ 1.778,0 , 1.0 ]]
    wing.aerodynamic_center                   = [ 1.8 ,0 , 1.0 ]    
    wing.winglet_fraction                     = 0.0  
    wing.symmetric                            = True
    wing.vertical                             = False
                                              
    # Segment                                              
    segment                                   = SUAVE.Components.Wings.Segment()
    segment.tag                               = 'Section_1'   
    segment.percent_span_location             = 0.0
    segment.twist                             = 0.0 
    segment.root_chord_percent                = 1 
    segment.dihedral_outboard                 = 8.  * Units.degrees
    segment.sweeps.quarter_chord              = 0. * Units.degrees 
    segment.thickness_to_chord                = 0.15 
    wing.Segments.append(segment)                           
                                              
    # Segment                                               
    segment                                   = SUAVE.Components.Wings.Segment()
    segment.tag                               = 'Section_2'    
    segment.percent_span_location             = 0.4875
    segment.twist                             = 0.0
    segment.root_chord_percent                = 0.6496
    segment.dihedral_outboard                 = 0. * Units.degrees
    segment.sweeps.quarter_chord              = 0. * Units.degrees
    segment.thickness_to_chord                = 0.135
    wing.Segments.append(segment)                                 
                                              
    # Segment                                              
    segment                                   = SUAVE.Components.Wings.Segment()
    segment.tag                               = 'Section_5'   
    segment.percent_span_location             = 1.0
    segment.twist                             = 0. 
    segment.root_chord_percent                = 0.42038
    segment.dihedral_outboard                 = 0.  * Units.degrees 
    segment.sweeps.quarter_chord              = 0.  * Units.degrees 
    segment.thickness_to_chord                = 0.12
    wing.Segments.append(segment)                 
                                              
    # add to vehicle                          
    vehicle.append_component(wing)                   
                                              
    # WING PROPERTIES                         
    wing                                      = SUAVE.Components.Wings.Wing()
    wing.tag                                  = 'v_tail'  
    wing.aspect_ratio                         = 4.27172 
    wing.sweeps.quarter_chord                 = 22.46  * Units.degrees 
    wing.thickness_to_chord                   = 0.15 
    wing.spans.projected                      = 3.6
    wing.chords.root                          = 1.193 
    wing.total_length                         = 1.193 
    wing.chords.tip                           = 0.535 
    wing.taper                                = 0.44  
    wing.chords.mean_aerodynamic              = 0.864 
    wing.dihedral                             = 45.0 * Units.degrees 
    wing.areas.reference                      = 4.25 * 2 
    wing.areas.wetted                         = 4.25 * 2 
    wing.areas.exposed                        = 4.25 * 2 
    wing.twists.root                          = 0 * Units.degrees 
    wing.twists.tip                           = 0 * Units.degrees 
    wing.origin                               = [[ 5.167, 0.0 ,0.470 ]]
    wing.aerodynamic_center                   = [  5.267,  0., 0.470  ]  
    wing.winglet_fraction                     = 0.0 
    wing.symmetric                            = True    

    # add to vehicle
    vehicle.append_component(wing)    
  
    # ---------------------------------------------------------------   
    # FUSELAGE                
    # ---------------------------------------------------------------   
    # FUSELAGE PROPERTIES
    fuselage                                    = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 2.  
    fuselage.seat_pitch                         = 3.  
    fuselage.fineness.nose                      = 0.88   
    fuselage.fineness.tail                      = 1.13   
    fuselage.lengths.nose                       = 0.5  
    fuselage.lengths.tail                       = 1.5
    fuselage.lengths.cabin                      = 4.46 
    fuselage.lengths.total                      = 6.46
    fuselage.width                              = 5.85 * Units.feet      # change 
    fuselage.heights.maximum                    = 4.65 * Units.feet      # change 
    fuselage.heights.at_quarter_length          = 3.75 * Units.feet      # change 
    fuselage.heights.at_wing_root_quarter_chord = 4.65 * Units.feet      # change 
    fuselage.heights.at_three_quarters_length   = 4.26 * Units.feet      # change 
    fuselage.areas.wetted                       = 236. * Units.feet**2   # change 
    fuselage.areas.front_projected              = 0.14 * Units.feet**2   # change 
    fuselage.effective_diameter                 = 1.276     # change 
    fuselage.differential_pressure              = 0. 
    
    # Segment  
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0 
    segment.percent_z_location                  = 0.     # change  
    segment.height                              = 0.049 
    segment.width                               = 0.032 
    fuselage.Segments.append(segment)                     
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_1'   
    segment.percent_x_location                  = 0.026  
    segment.percent_z_location                  = 0.00849
    segment.height                              = 0.481 
    segment.width                               = 0.553 
    fuselage.Segments.append(segment)           
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.074
    segment.percent_z_location                  = 0.02874
    segment.height                              = 1.00
    segment.width                               = 0.912 
    fuselage.Segments.append(segment)                     
                                                
    # Segment                                            
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.161  
    segment.percent_z_location                  = 0.04348   
    segment.height                              = 1.41
    segment.width                               = 1.174  
    fuselage.Segments.append(segment)                     
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.284 
    segment.percent_z_location                  = 0.05435 
    segment.height                              = 1.62
    segment.width                               = 1.276  
    fuselage.Segments.append(segment)              
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.531 
    segment.percent_z_location                  = 0.0510 
    segment.height                              = 1.409
    segment.width                               = 1.121 
    fuselage.Segments.append(segment)                     
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.651
    segment.percent_z_location                  = 0.05636 
    segment.height                              = 1.11
    segment.width                               = 0.833
    fuselage.Segments.append(segment)                  
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.773
    segment.percent_z_location                  = 0.06149 
    segment.height                              = 0.78
    segment.width                               = 0.512 
    fuselage.Segments.append(segment)                  
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 1.
    segment.percent_z_location                  = 0.07352  
    segment.height                              = 0.195  
    segment.width                               = 0.130 
    fuselage.Segments.append(segment)                   
                                                
    vehicle.append_component(fuselage) 


    #------------------------------------------------------------------
    # PROPULSOR
    #------------------------------------------------------------------
    net                               = Lift_Cruise()
    net.number_of_lift_rotor_engines  = 4
    net.number_of_propeller_engines   = 2
    net.number_of_engines             = net.number_of_lift_rotor_engines+ net.number_of_propeller_engines
    net.rotor_thrust_angle            = 90. * Units.degrees
    net.propeller_thrust_angle        = 0.    
    net.identical_propellers = True
    net.identical_rotors     = True

    # Nacelles 
    nacelle                           = SUAVE.Components.Nacelles.Nacelle()
    nacelle.tag                       = 'rotor_nacelle'
    nacelle.length                    = 1.5
    nacelle.diameter                  = 0.5
    net.areas.wetted                  = np.pi*nacelle.diameter*nacelle.length  + 0.5*np.pi*nacelle.diameter**2  
    nacelle.orientation_euler_angles  = [0.,0.,0.]    
    nacelle.flow_through              = False  
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_1'
    nac_segment.percent_x_location = 0.0  
    nac_segment.height             = 0.2
    nac_segment.width              = 0.2
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


    lift_rotor_nacelle_origins = [[1.305,5.000,1.320],[1.305,-5.000,1.320],[   5.217, -1.848,   2.282],[   5.217, 1.848,   2.282]] 
    
    for ii in range(4):
        rotor_nacelle          = deepcopy(nacelle)
        rotor_nacelle.tag      = 'lift_rotor_nacelle_' + str(ii+1) 
        rotor_nacelle.origin   = [lift_rotor_nacelle_origins[ii]]
        vehicle.append_component(rotor_nacelle)       
    
    
    propeller_nacelle_origins = [[ -0.109, -2.392,  1.195],[ -0.109, 2.392,  1.195]]     
    for ii in range(2):
        rotor_nacelle          = deepcopy(nacelle)
        rotor_nacelle.length   = 3.
        rotor_nacelle.tag      = 'propeller_nacelle_' + str(ii+1) 
        rotor_nacelle.origin   = [propeller_nacelle_origins[ii]]
        vehicle.append_component(rotor_nacelle)           
        
    # Component 1 the ESC
    rotor_esc                       = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    rotor_esc.efficiency            = 0.95
    net.rotor_esc                   = rotor_esc 
                                    
    propeller_esc                   = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    propeller_esc.efficiency        = 0.95
    net.propeller_esc               = propeller_esc
 
    # Component 2 the Payload
    payload                         = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw              = 10. #Watts 
    payload.mass_properties.mass    = 1.0 * Units.kg
    net.payload                     = payload
                                    
    # Component 3 the Avionics      
    avionics                        = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw             = 20. #Watts  
    net.avionics                    = avionics    
    
    # Component 4 Miscellaneous Systems 
    sys                             = SUAVE.Components.Systems.System()
    sys.mass_properties.mass        = 5 # kg
  
    # Component 5 the Battery
    bat                            = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650()    
    bat.pack_config.series         =  100  
    bat.pack_config.parallel       =  120    
    initialize_from_circuit_configuration(bat)       
    net.battery                    = bat 
    net.voltage                    = bat.max_voltage     

    # Component 6 the Rotor    
    g                                     = 9.81                                   # gravitational acceleration   
    rho                                   = 1.22                                   # reference density 
    Hover_Load                            = vehicle.mass_properties.takeoff*g      # hover load   
    prop_rotor                            = SUAVE.Components.Energy.Converters.Lift_Rotor()   
    prop_rotor.tip_radius                 = 3/2    
    prop_rotor.hub_radius                 = 0.15*prop_rotor.tip_radius
    prop_rotor.number_of_blades           = 4 
    prop_rotor.optimization_parameters.tip_mach_range = [0.3,0.5] 
    prop_rotor.number_of_engines          = net.number_of_lift_rotor_engines
    prop_rotor.disc_area                  = np.pi*(prop_rotor.tip_radius**2)        
    prop_rotor.induced_hover_velocity     = np.sqrt(Hover_Load/(2*rho*prop_rotor.disc_area*net.number_of_engines ))   
    prop_rotor.design_thrust              = (Hover_Load)/(net.number_of_engines-1)
    prop_rotor.freestream_velocity        = np.sqrt(prop_rotor.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))      
    prop_rotor.design_Cl                  = 0.7
    prop_rotor.design_altitude            = 40 * Units.feet 
    prop_rotor.symmetric                  = True     
    prop_rotor_origins                    = [[0.208, -2.392,  1.195],[0.208, 2.392,  1.195]]
    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    rel_path                              = os.path.dirname(ospath) + separator  
    airfoil                               = SUAVE.Components.Airfoils.Airfoil()    
    airfoil.coordinate_file               =  rel_path +'../Airfoils/NACA_4412.txt'
    airfoil.polar_files                   = [rel_path +'../Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                             rel_path +'../Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                             rel_path +'../Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                             rel_path +'../Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                             rel_path +'../Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ] 
    prop_rotor.append_airfoil(airfoil)   
    prop_rotor.airfoil_polar_stations          = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]      
    prop_rotor                            =  lift_rotor_design(prop_rotor)  
    prop_rotor.orientation_euler_angles   = [0, 0*Units.degrees,0]  
    for ii in range(2):
        lift_rotor                        = deepcopy(prop_rotor)
        lift_rotor.tag                    = 'lift_rotor_' + str(ii+1) 
        lift_rotor.origin                 = [prop_rotor_origins[ii]] 
        net.propellers.append(lift_rotor)   
                                          
    # Lift Rotors                                  
    rotor                                 = deepcopy(prop_rotor) 
    rotor.tag                             = 'Lift_rotor'
    lift_rotor_origins                    = [[1.505,5.000,1.320],[1.505,-5.000,1.320],[  5.318, 1.848,   2.282],[   5.318, -1.848,   2.282]]    
    for ii in range(4):
        lift_rotor                        = deepcopy(rotor)
        lift_rotor.tag                    = 'lift_rotor_' + str(ii+1) 
        lift_rotor.origin                 = [lift_rotor_origins[ii]] 
        net.lift_rotors.append(lift_rotor) 
        

    # Component 7 the Motors
    # Propeller (Thrust) motor
    prop_rotor_motor                      = SUAVE.Components.Energy.Converters.Motor() 
    prop_rotor_motor.nominal_voltage      = bat.max_voltage*3/4   
    prop_rotor_motor.efficiency           = 0.9
    prop_rotor_motor.origin               = prop_rotor.origin  
    prop_rotor_motor.propeller_radius     = prop_rotor.tip_radius      
    prop_rotor_motor.no_load_current      = 0.01  
    prop_rotor_motor                      = size_optimal_motor(prop_rotor_motor,prop_rotor)
    prop_rotor_motor.mass_properties.mass = nasa_motor(prop_rotor_motor.design_torque) 

    # Appending motors with different origins
    for ii in range(4):
        motor        = deepcopy(prop_rotor_motor)
        motor.tag    = 'lift_rotor_motor_' + str(ii+1)
        motor.origin =  [lift_rotor_origins[ii]]                     
        net.lift_rotor_motors.append(motor) 
        
    for ii in range(2):
        motor = deepcopy(prop_rotor_motor)
        motor.tag = 'propeller_motor_' + str(ii+1)
        motor.origin =  [prop_rotor_origins[ii]]    
        net.propeller_motors.append(motor) 
         

    # Rotor (Lift) Motor                         
    net.rotor_motor                       = prop_rotor_motor
    vehicle.append_component(net)

    # Add extra drag sources from motors, props, and landing gear. All of these hand measured 
    motor_height                          = .25 * Units.feet
    motor_width                           = 1.6 * Units.feet    
    propeller_width                       = 1. * Units.inches
    propeller_height                      = propeller_width *.12    
    main_gear_width                       = 1.5 * Units.inches
    main_gear_length                      = 2.5 * Units.feet    
    nose_gear_width                       = 2. * Units.inches
    nose_gear_length                      = 2. * Units.feet    
    nose_tire_height                      = (0.7 + 0.4) * Units.feet
    nose_tire_width                       = 0.4 * Units.feet    
    main_tire_height                      = (0.75 + 0.5) * Units.feet
    main_tire_width                       = 4. * Units.inches    
    total_excrescence_area_spin           = 12.*motor_height*motor_width + 2.*main_gear_length*main_gear_width \
        + nose_gear_width*nose_gear_length + 2*main_tire_height*main_tire_width\
        + nose_tire_height*nose_tire_width 
    total_excrescence_area_no_spin        = total_excrescence_area_spin + 12*propeller_height*propeller_width  
    vehicle.excrescence_area_no_spin      = total_excrescence_area_no_spin 
    vehicle.excrescence_area_spin         = total_excrescence_area_spin  
    
    main_wing_motor_origins                 = np.array([[-0.109,2.283 ,1.630],[-0.109,-2.283 ,1.630] ,[ 2.283,4.891,2.391],[ 2.283,-4.891,2.391]]) 
    tail_motor_origins                      = np.array([[6.522,2.174,2.065],[6.522,-2.174,2.065]]) 
    vehicle.wings['main_wing'].motor_spanwise_locations = main_wing_motor_origins[:,1]/vehicle.wings['main_wing'].spans.projected
    vehicle.wings['v_tail'].motor_spanwise_locations    = tail_motor_origins[:,1]/vehicle.wings['main_wing'].spans.projected
   
    #converge_evtol_weight(vehicle)
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
    aerodynamics.settings.drag_coefficient_increment = 0.0
    analyses.append(aerodynamics)

    ## ------------------------------------------------------------------
    ##  Noise Analysis
    #noise = SUAVE.Analyses.Noise.Fidelity_One()   
    #noise.geometry = vehicle
    #analyses.append(noise)
    
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
    base_segment                                             = Segments.Segment()
    ones_row                                                 = base_segment.state.ones_row
    base_segment.use_Jacobian                                = False  
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip
    base_segment.process.iterate.unknowns.network            = vehicle.networks.lift_cruise.unpack_unknowns_transition
    base_segment.process.iterate.residuals.network           = vehicle.networks.lift_cruise.residuals_transition
    base_segment.state.unknowns.battery_voltage_under_load   = vehicle.networks.lift_cruise.battery.max_voltage * ones_row(1)  
    base_segment.state.residuals.network                     = 0. * ones_row(2)    


    # VSTALL Calculation
    m      = vehicle.mass_properties.max_takeoff
    g      = 9.81
    S      = vehicle.reference_area
    atmo   = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    rho    = atmo.compute_values(1000.*Units.feet,0.).density
    CLmax  = 1.2 
    Vstall = float(np.sqrt(2.*m*g/(rho*S*CLmax)))

    # ------------------------------------------------------------------
    #   First Taxi Segment: Constant Speed
    # ------------------------------------------------------------------

    segment = Segments.Hover.Climb(base_segment)
    segment.tag = "Ground_Taxi"

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment     =  Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
    segment.tag = "climb_1" 
    
    segment.analyses.extend( analyses.hover_climb)  
    
    segment.analyses.extend( analyses.transition_fast ) 
    segment.altitude_start         = 0.0 * Units.ft
    segment.altitude_end           = 40.0 * Units.ft
    segment.air_speed              = 0. * Units['ft/min'] 
    segment.climb_angle            = 90 * Units.degrees
    segment.acceleration           = 0.5 * Units['m/s/s']    
    segment.pitch_initial          = 0. * Units.degrees  
    segment.pitch_final            = 0. * Units.degrees      
    
    segment.battery_energy                                   = vehicle.networks.lift_cruise.battery.max_energy
                                                             
    segment.state.unknowns.rotor_power_coefficient           = 0.016 * ones_row(1) 
    segment.state.unknowns.throttle_lift                     = 0.9   * ones_row(1) 
    segment.state.unknowns.propeller_power_coefficient       = 0.016 * ones_row(1) 
    segment.state.unknowns.throttle                          = 0.9   * ones_row(1) 
    segment.state.residuals.network                          = 0.    * ones_row(3)

    segment.process.iterate.unknowns.network                 = vehicle.networks.lift_cruise.unpack_unknowns_transition
    segment.process.iterate.residuals.network                = vehicle.networks.lift_cruise.residuals_transition    
    segment.process.iterate.unknowns.mission                 = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability             = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability          = SUAVE.Methods.skip 

    # add to misison
    mission.append_segment(segment)
    
    ## ------------------------------------------------------------------
    ##   First Cruise Segment: Transition
    ## ------------------------------------------------------------------

    #segment     = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
    #segment.tag = "transition_1"

    #segment.analyses.extend( analyses.transition_slow ) 
    #segment.altitude        = 40.  * Units.ft
    #segment.air_speed_start = 500. * Units['ft/min']
    #segment.air_speed_end   = 0.8 * Vstall
    #segment.acceleration    = 9.8/5
    #segment.pitch_initial   = 0.0 * Units.degrees
    #segment.pitch_final     = 5. * Units.degrees
    
    #segment.state.unknowns.rotor_power_coefficient          = 0.016 * ones_row(1) 
    #segment.state.unknowns.throttle_lift                    = 0.9   * ones_row(1) 
    #segment.state.unknowns.propeller_power_coefficient      = 0.016 * ones_row(1) 
    #segment.state.unknowns.throttle                         = 0.9   * ones_row(1) 
    #segment.state.residuals.network                         = 0.    * ones_row(3)

    #segment.process.iterate.unknowns.network                = vehicle.networks.lift_cruise.unpack_unknowns_transition
    #segment.process.iterate.residuals.network               = vehicle.networks.lift_cruise.residuals_transition    
    #segment.process.iterate.unknowns.mission                = SUAVE.Methods.skip
    #segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    #segment.process.finalize.post_process.stability         = SUAVE.Methods.skip 
    
    ## add to misison
    #mission.append_segment(segment)
    
    ## ------------------------------------------------------------------
    ##   First Cruise Segment: Transition
    ## ------------------------------------------------------------------

    #segment     = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
    #segment.tag = "transition_2"

    #segment.analyses.extend( analyses.transition_fast ) 
    #segment.altitude_start         = 40.0 * Units.ft
    #segment.altitude_end           = 50.0 * Units.ft
    #segment.air_speed              = 0.8 * Vstall
    #segment.climb_angle            = 1 * Units.degrees
    #segment.acceleration           = 0.5 * Units['m/s/s']    
    #segment.pitch_initial          = 5. * Units.degrees  
    #segment.pitch_final            = 7. * Units.degrees       
    
    #segment.state.unknowns.rotor_power_coefficient          = 0.016 * ones_row(1) 
    #segment.state.unknowns.throttle_lift                    = 0.9   * ones_row(1) 
    #segment.state.unknowns.propeller_power_coefficient      = 0.016 * ones_row(1) 
    #segment.state.unknowns.throttle                         = 0.9   * ones_row(1) 
    #segment.state.residuals.network                         = 0.    * ones_row(3)

    #segment.process.iterate.unknowns.network                = vehicle.networks.lift_cruise.unpack_unknowns_transition
    #segment.process.iterate.residuals.network               = vehicle.networks.lift_cruise.residuals_transition    
    #segment.process.iterate.unknowns.mission                = SUAVE.Methods.skip
    #segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    #segment.process.finalize.post_process.stability         = SUAVE.Methods.skip

    ## add to misison
    #mission.append_segment(segment)
    
  
    ## ------------------------------------------------------------------
    ##   Second Climb Segment: Constant Speed, Constant Rate
    ## ------------------------------------------------------------------

    #segment     = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    #segment.tag = "climb_2"

    #segment.analyses.extend( analyses.cruise )

    #segment.air_speed       = 1.1*Vstall
    #segment.altitude_start  = 50.0 * Units.ft
    #segment.altitude_end    = 300. * Units.ft
    #segment.climb_rate      = 500. * Units['ft/min']

    #segment.state.unknowns.propeller_power_coefficient         = 0.02   * ones_row(1)
    #segment.state.unknowns.throttle                            = 0.60   * ones_row(1)
    #segment.process.iterate.unknowns.network  = vehicle.networks.lift_cruise.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.networks.lift_cruise.residuals_no_lift     

    ## add to misison
    #mission.append_segment(segment)

    ## ------------------------------------------------------------------
    ##   Second Cruise Segment: Constant Speed, Constant Altitude
    ## ------------------------------------------------------------------

    #segment     = Segments.Cruise.Constant_Speed_Constant_Altitude_Loiter(base_segment)
    #segment.tag = "departure_terminal_procedures"

    #segment.analyses.extend( analyses.cruise )

    #segment.altitude  = 300.0 * Units.ft
    #segment.time      = 60.   * Units.second
    #segment.air_speed = 1.2*Vstall

    #segment.state.unknowns.propeller_power_coefficient = 0.02  * ones_row(1)
    #segment.state.unknowns.throttle                    = 0.60  * ones_row(1)

    #segment.process.iterate.unknowns.network  = vehicle.networks.lift_cruise.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.networks.lift_cruise.residuals_no_lift     


    ## add to misison
    #mission.append_segment(segment)

    ## ------------------------------------------------------------------
    ##   Third Climb Segment: Constant Acceleration, Constant Rate
    ## ------------------------------------------------------------------

    #segment     = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    #segment.tag = "accelerated_climb"

    #segment.analyses.extend( analyses.cruise )

    #segment.altitude_start  = 300.0 * Units.ft
    #segment.altitude_end    = 2500. * Units.ft
    #segment.climb_rate      = 500.  * Units['ft/min']
    #segment.air_speed_start = np.sqrt((500 * Units['ft/min'])**2 + (1.2*Vstall)**2)
    #segment.air_speed_end   = 110.  * Units['mph']                                            

    #segment.state.unknowns.propeller_power_coefficient =  0.02   * ones_row(1)
    #segment.state.unknowns.throttle                    =  0.60   * ones_row(1)

    #segment.process.iterate.unknowns.network  = vehicle.networks.lift_cruise.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.networks.lift_cruise.residuals_no_lift   

    ## add to misison
    #mission.append_segment(segment)    

    ## ------------------------------------------------------------------
    ##   Third Cruise Segment: Constant Acceleration, Constant Altitude
    ## ------------------------------------------------------------------

    #segment     = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    #segment.tag = "cruise"

    #segment.analyses.extend( analyses.cruise )
 
    #segment.air_speed = 110.   * Units['mph']
    #segment.distance  = 60.    * Units.miles                       

    #segment.state.unknowns.propeller_power_coefficient = 0.012 * ones_row(1)  
    #segment.state.unknowns.throttle                    = 0.85  * ones_row(1)

    #segment.process.iterate.unknowns.network  = vehicle.networks.lift_cruise.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.networks.lift_cruise.residuals_no_lift    


    ## add to misison
    #mission.append_segment(segment)     
  

    ## ------------------------------------------------------------------
    ##   First Descent Segment: Constant Acceleration, Constant Rate
    ## ------------------------------------------------------------------

    #segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    #segment.tag = "decelerating_descent"

    #segment.analyses.extend( analyses.cruise )  
    #segment.altitude_start  = 2500.0 * Units.ft
    #segment.altitude_end    = 300. * Units.ft
    #segment.climb_rate      = -300.  * Units['ft/min']
    #segment.air_speed_start = 110.  * Units['mph']
    #segment.air_speed_end   = 1.2*Vstall

    #segment.state.unknowns.propeller_power_coefficient =  0.01 * ones_row(1)
    #segment.state.unknowns.throttle                    =  0.80 * ones_row(1)

    #segment.process.iterate.unknowns.network  = vehicle.networks.lift_cruise.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.networks.lift_cruise.residuals_no_lift     

    ## add to misison
    #mission.append_segment(segment)        

    ## ------------------------------------------------------------------
    ##   Reserve Segment: Constant Speed, Constant Altitude
    ## ------------------------------------------------------------------

    #segment = Segments.Cruise.Constant_Speed_Constant_Altitude_Loiter(base_segment)
    #segment.tag = "arrival_terminal_procedures"

    #segment.analyses.extend( analyses.cruise )

    #segment.altitude        = 300.   * Units.ft
    #segment.air_speed       = 1.2*Vstall
    #segment.time            = 60 * Units.seconds

    #segment.state.unknowns.propeller_power_coefficient = 0.01 * ones_row(1)
    #segment.state.unknowns.throttle                    = 0.80 * ones_row(1)

    #segment.process.iterate.unknowns.network  = vehicle.networks.lift_cruise.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.networks.lift_cruise.residuals_no_lift   

    ## add to misison
    #mission.append_segment(segment)    
    
    
    ## ------------------------------------------------------------------
    ##   Reserve Segment: Constant Acceleration, Constant Rate
    ## ------------------------------------------------------------------

    #segment     = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    #segment.tag = "reserve_accelerated_climb"

    #segment.analyses.extend( analyses.cruise )

    #segment.altitude_start  = 300.0 * Units.ft
    #segment.altitude_end    = 1500. * Units.ft
    #segment.climb_rate      = 500.  * Units['ft/min']
    #segment.air_speed_start = np.sqrt((500 * Units['ft/min'])**2 + (1.2*Vstall)**2)
    #segment.air_speed_end   = 110.  * Units['mph']                                            

    #segment.state.unknowns.propeller_power_coefficient =  0.01   * ones_row(1)
    #segment.state.unknowns.throttle                    =  0.80   * ones_row(1)

    #segment.process.iterate.unknowns.network  = vehicle.networks.lift_cruise.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.networks.lift_cruise.residuals_no_lift   

    ## add to misison
    #mission.append_segment(segment)    

    ## ------------------------------------------------------------------
    ##   Reserve Cruise Segment: Constant Acceleration, Constant Altitude
    ## ------------------------------------------------------------------

    #segment     = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)   
    #segment.tag = "reserve_cruise"

    #segment.analyses.extend( analyses.cruise )
 
    #segment.altitude  = 1500.0 * Units.ft
    #segment.air_speed = 110.   * Units['mph']
    #segment.distance  = 33.5    * Units.miles  * 0.1                          

    #segment.state.unknowns.propeller_power_coefficient = 0.012  * ones_row(1)  
    #segment.state.unknowns.throttle                    = 0.85   * ones_row(1)

    #segment.process.iterate.unknowns.network  = vehicle.networks.lift_cruise.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.networks.lift_cruise.residuals_no_lift    


    ## add to misison
    #mission.append_segment(segment)     
  

    ## ------------------------------------------------------------------
    ##   Reserve Descent Segment: Constant Acceleration, Constant Rate
    ## ------------------------------------------------------------------

    #segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    #segment.tag = "reserve_decelerating_descent"

    #segment.analyses.extend( analyses.cruise )  
    #segment.altitude_start  = 1500.0 * Units.ft
    #segment.altitude_end    = 300. * Units.ft
    #segment.climb_rate      = -300.  * Units['ft/min']
    #segment.air_speed_start = 110.  * Units['mph']
    #segment.air_speed_end   = 1.2*Vstall

    #segment.state.unknowns.propeller_power_coefficient =  0.01  * ones_row(1)
    #segment.state.unknowns.throttle                    =  0.80  * ones_row(1)

    #segment.process.iterate.unknowns.network  = vehicle.networks.lift_cruise.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.networks.lift_cruise.residuals_no_lift     

    ## add to misison
    #mission.append_segment(segment)        

    ## ------------------------------------------------------------------
    ##  Reserve Cruise Segment: Constant Speed, Constant Altitude
    ## ------------------------------------------------------------------

    #segment = Segments.Cruise.Constant_Speed_Constant_Altitude_Loiter(base_segment)
    #segment.tag = "arrival_terminal_procedures"

    #segment.analyses.extend( analyses.cruise )

    #segment.altitude        = 300.   * Units.ft
    #segment.air_speed       = 1.2*Vstall
    #segment.time            = 60 * Units.seconds 

    #segment.state.unknowns.propeller_power_coefficient = 0.01 * ones_row(1)
    #segment.state.unknowns.throttle                    = 0.80 * ones_row(1)

    #segment.process.iterate.unknowns.network  = vehicle.networks.lift_cruise.unpack_unknowns_no_lift
    #segment.process.iterate.residuals.network = vehicle.networks.lift_cruise.residuals_no_lift   

    ## add to misison
    #mission.append_segment(segment)   
    
    ## ------------------------------------------------------------------
    ##   Second Descent Segment: Constant Speed, Constant Rate
    ## ------------------------------------------------------------------  
    #segment     = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)   
    #segment.tag = "descent_transition"

    #segment.analyses.extend( analyses.transition_fast ) 
    #segment.altitude_start         = 300.0 * Units.ft
    #segment.altitude_end           = 40.   * Units.ft
    #segment.air_speed              = 1.2   * Vstall  
    #segment.climb_angle            = 3.    * Units.degrees
    #segment.acceleration           = -0.02  * Units['m/s/s']    
    #segment.pitch_initial          = 7.    * Units.degrees  
    #segment.pitch_final            = 5.    * Units.degrees       
    
    #segment.state.unknowns.rotor_power_coefficient          = 0.02  * ones_row(1) 
    #segment.state.unknowns.throttle_lift                    = 0.6   * ones_row(1) 
    #segment.state.unknowns.propeller_power_coefficient      = 0.01  * ones_row(1) 
    #segment.state.unknowns.throttle                         = 0.80  * ones_row(1) 
    #segment.state.residuals.network                         = 0.    * ones_row(3)
     
    #segment.process.iterate.unknowns.network                = vehicle.networks.lift_cruise.unpack_unknowns_transition
    #segment.process.iterate.residuals.network               = vehicle.networks.lift_cruise.residuals_transition    
    #segment.process.iterate.unknowns.mission                = SUAVE.Methods.skip
    #segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    #segment.process.finalize.post_process.stability         = SUAVE.Methods.skip

    ## add to misison
    #mission.append_segment(segment)
        

    ## ------------------------------------------------------------------
    ##   Fifth Cuise Segment: Transition
    ## ------------------------------------------------------------------ 
    #segment = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)  
    #segment.tag = "final_transition"
 
    #segment.analyses.extend( analyses.transition_slow )

    #segment.altitude        = 40. * Units.ft
    #segment.air_speed_start = 1.2 * Vstall      
    #segment.air_speed_end   = 300. * Units['ft/min'] 
    #segment.acceleration    = -9.81/20
    #segment.pitch_initial   = 5. * Units.degrees   
    #segment.pitch_final     = 10. * Units.degrees      
  
    #segment.state.unknowns.rotor_power_coefficient          = 0.07  *  ones_row(1) 
    #segment.state.unknowns.throttle_lift                    = 0.75  *  ones_row(1) 
    #segment.state.unknowns.propeller_power_coefficient      = 0.01  *  ones_row(1)   
    #segment.state.unknowns.throttle                         = 0.75   *  ones_row(1)   
    #segment.state.residuals.network                         = 0.    * ones_row(3)    
    
    #segment.process.iterate.unknowns.network  = vehicle.networks.lift_cruise.unpack_unknowns_transition
    #segment.process.iterate.residuals.network = vehicle.networks.lift_cruise.residuals_transition    
    #segment.process.iterate.unknowns.mission  = SUAVE.Methods.skip 
    ## add to misison
    #mission.append_segment(segment)  
 
    ## ------------------------------------------------------------------
    ##   Third Descent Segment: Constant Speed, Constant Rate
    ## ------------------------------------------------------------------ 
    #segment     =  Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
    #segment.tag = "descent_1" 
    
    #segment.analyses.extend( analyses.hover_climb)  
    
    #segment.analyses.extend( analyses.transition_fast ) 
    #segment.altitude_start         = 40.0  * Units.ft
    #segment.altitude_end           = 0.0   * Units.ft
    #segment.air_speed              = 300. * Units['ft/min'] 
    #segment.climb_angle            = 90 * Units.degrees
    #segment.acceleration           = 0.1 * Units['m/s/s']    
    #segment.pitch_initial          = 0. * Units.degrees  
    #segment.pitch_final            = 0. * Units.degrees       
                                                             
    #segment.state.unknowns.rotor_power_coefficient           = 0.08   * ones_row(1) 
    #segment.state.unknowns.throttle_lift                     = 0.9   * ones_row(1) 
    #segment.state.unknowns.propeller_power_coefficient       = 0.08   * ones_row(1) 
    #segment.state.unknowns.throttle                          = 0.9   * ones_row(1) 
    #segment.state.residuals.network                          = 0.    * ones_row(3)

    #segment.process.iterate.unknowns.network                 = vehicle.networks.lift_cruise.unpack_unknowns_transition
    #segment.process.iterate.residuals.network                = vehicle.networks.lift_cruise.residuals_transition    
    #segment.process.iterate.unknowns.mission                 = SUAVE.Methods.skip
    #segment.process.iterate.conditions.stability             = SUAVE.Methods.skip
    #segment.process.finalize.post_process.stability          = SUAVE.Methods.skip 

    ## add to misison
    #mission.append_segment(segment)
    
  
    return mission


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
    config                                                 = SUAVE.Components.Configs.Config(base_config)
    config.tag                                             = 'hover' 
    vector_angle                                           = 90.0 * Units.degrees
    config.networks.lift_cruise.propeller_thrust_angle   = vector_angle 
    config.networks.lift_cruise.propeller_pitch_command  = 0.  * Units.degrees 
    configs.append(config)         

    # ------------------------------------------------------------------
    #   Hover Climb Configuration
    # ------------------------------------------------------------------
    config                                                 = SUAVE.Components.Configs.Config(base_config)
    config.tag                                             = 'hover_climb'
    vector_angle                                           = 90.0 * Units.degrees
    config.networks.lift_cruise.propeller_thrust_angle   = vector_angle 
    config.networks.lift_cruise.propeller_pitch_command  = -5.  * Units.degrees  
    configs.append(config)

    # ------------------------------------------------------------------
    #   Hover-to-Cruise Configuration
    # ------------------------------------------------------------------
    config                                                 = SUAVE.Components.Configs.Config(base_config)
    config.tag                                             = 'transition_slow'
    vector_angle                                           = 85.0 * Units.degrees
    config.networks.lift_cruise.propeller_thrust_angle   = vector_angle 
    config.networks.lift_cruise.propeller_pitch_command  = 0.  * Units.degrees   
    configs.append(config)

    # ------------------------------------------------------------------
    #   Hover-to-Cruise Configuration
    # ------------------------------------------------------------------
    config                                                 = SUAVE.Components.Configs.Config(base_config)
    config.tag                                             = 'transition_fast' 
    vector_angle                                           = 65.0 * Units.degrees
    config.networks.lift_cruise.propeller_thrust_angle   = vector_angle 
    config.networks.lift_cruise.propeller_pitch_command  = 2.  * Units.degrees     
    configs.append(config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config                                                 = SUAVE.Components.Configs.Config(base_config)
    config.tag                                             = 'cruise'   
    vector_angle                                           = 0.0 * Units.degrees
    config.networks.lift_cruise.propeller_thrust_angle   = vector_angle 
    config.networks.lift_cruise.propeller_pitch_command  = 10.  * Units.degrees  
    configs.append(config)     

    return configs 



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
def plot_mission(results,line_style='bo-'): 
    
    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style) 
    
    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results, line_style)  
    
    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results, line_style) 
    
    # Plot Electric Motor and Propeller Efficiencies  of Lift Cruise Network
    plot_lift_cruise_network(results, line_style)  
    return     

def save_results(results):

    # Store data (serialize)
    with open('Stopped_Rotor.pkl', 'wb') as file:
        pickle.dump(results, file)
        
    return  

if __name__ == '__main__': 
    main()    
    plt.show()
     