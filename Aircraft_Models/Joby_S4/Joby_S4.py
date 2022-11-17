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
from SUAVE.Components.Energy.Networks.Battery_Propeller                      import Battery_Propeller 
from SUAVE.Methods.Power.Battery.Sizing                                      import initialize_from_circuit_configuration  
from SUAVE.Methods.Weights.Correlations.Propulsion                           import nasa_motor
from SUAVE.Methods.Propulsion.electric_motor_sizing                          import size_optimal_motor
from SUAVE.Methods.Propulsion                                                import prop_rotor_design  
from SUAVE.Methods.Weights.Buildups.eVTOL.empty                              import empty
from SUAVE.Methods.Center_of_Gravity.compute_component_centers_of_gravity    import compute_component_centers_of_gravity
from SUAVE.Methods.Geometry.Two_Dimensional.Planform                         import segment_properties
from SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_segmented_planform import wing_segmented_planform 
from SUAVE.Methods.Weights.Buildups.eVTOL.converge_evtol_weight              import converge_evtol_weight  

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
    
    configs.finalize()
    analyses.finalize()     
    
    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()
 
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
    #write(vehicle, "Joby_S4") 
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
    


    # compute reference properties 
    wing_segmented_planform(wing, overwrite_reference = True ) 
    wing = segment_properties(wing)
    vehicle.reference_area        = wing.areas.reference  
    wing.areas.wetted             = wing.areas.reference  * 2 
    wing.areas.exposed            = wing.areas.reference  * 2  
        
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
    net                               = Battery_Propeller()  
    net.number_of_propeller_engines   = 6  
    net.engine_length                 = 1.5
    net.identical_propellers          = True 

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
    
    
    propeller_nacelle_origins = [[ -0.109, -1.848,  1.195],[ -0.109, 1.848,  1.195]]     
    for ii in range(2):
        rotor_nacelle          = deepcopy(nacelle)
        rotor_nacelle.length   = 3.
        rotor_nacelle.tag      = 'propeller_nacelle_' + str(ii+1) 
        rotor_nacelle.origin   = [propeller_nacelle_origins[ii]]
        vehicle.append_component(rotor_nacelle)    
    
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
    g                                     = 9.81                                   # gravitational acceleration    
    Hover_Load                            = vehicle.mass_properties.takeoff*g      # hover load  
    

    # DEFINE ROTOR OPERATING CONDITIONS  
    prop_rotor                                    = SUAVE.Components.Energy.Converters.Prop_Rotor() 
    prop_rotor.tag                                = 'prop_rotor'     
    prop_rotor.tip_radius                         = 3/2
    prop_rotor.hub_radius                         = 0.15 * prop_rotor.tip_radius
    prop_rotor.number_of_blades                   = 4
    
    # HOVER 
    prop_rotor.design_altitude_hover              = 20 * Units.feet                  
    prop_rotor.design_thrust_hover                = Hover_Load/6 # weight of joby-like aircrft
    prop_rotor.freestream_velocity_hover          = np.sqrt(prop_rotor.design_thrust_hover/(2*1.2*np.pi*(prop_rotor.tip_radius**2))) # 
 
    # CRUISE                   
    prop_rotor.design_altitude_cruise             = 2500 * Units.feet                      
    prop_rotor.design_thrust_cruise               = 4000/6
    prop_rotor.freestream_velocity_cruise         = 175*Units.mph 

    airfoil                                       = SUAVE.Components.Airfoils.Airfoil()    
    ospath                                        = os.path.abspath(__file__)
    separator                                     = os.path.sep
    rel_path                                      = os.path.dirname(ospath) + separator  
    airfoil                                       = SUAVE.Components.Airfoils.Airfoil()    
    airfoil.coordinate_file                       =  rel_path +'../Airfoils/NACA_4412.txt'
    airfoil.polar_files                           = [rel_path +'../Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                                     rel_path +'../Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                                     rel_path +'../Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                                     rel_path +'../Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                                     rel_path +'../Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ] 
    prop_rotor.append_airfoil(airfoil)   
    prop_rotor.airfoil_polar_stations            = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]   
    prop_rotor                                   = prop_rotor_design(prop_rotor)  
    prop_rotor.design_Cl                         = prop_rotor.design_Cl_hover
            
    # Prop Rotors                                          
    prop_rotor_origins           = [[0.208, -1.848,  1.195],[0.208, 1.848,  1.195],
                                    [1.505,5.000,1.320],[1.505,-5.000,1.320],
                                    [  5.318, 1.848,   2.282],[   5.318, -1.848,   2.282]]    
    for ii in range(6):
        pr                        = deepcopy(prop_rotor)
        pr.tag                    = 'prop_rotor_' + str(ii+1) 
        pr.origin                 = [prop_rotor_origins[ii]] 
        net.propellers.append(pr)  

    # Component 7 the Motors
    # Propeller (Thrust) motor
    prop_rotor_motor                      = SUAVE.Components.Energy.Converters.Motor() 
    prop_rotor_motor.nominal_voltage      = bat.max_voltage*0.8  
    prop_rotor_motor.efficiency           = 0.9
    prop_rotor_motor.origin               = prop_rotor.origin  
    prop_rotor_motor.propeller_radius     = prop_rotor.tip_radius      
    prop_rotor_motor.no_load_current      = 0.1  
    prop_rotor_motor                      = size_optimal_motor(prop_rotor_motor,prop_rotor)
    prop_rotor_motor.mass_properties.mass = nasa_motor(prop_rotor_motor.design_torque) 

    # Appending motors with different origins
    for ii in range(6):
        motor        = deepcopy(prop_rotor_motor)
        motor.tag    = 'prop_rotor_motor_' + str(ii+1)
        motor.origin =  [prop_rotor_origins[ii]]                     
        net.propeller_motors.append(motor)   
 
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
   

    converge_evtol_weight(vehicle,print_iterations=True,contingency_factor = 1.0 )
    settings = Data()
    vehicle.weight_breakdown  = empty(vehicle,settings,contingency_factor = 1.0 )
    compute_component_centers_of_gravity(vehicle)
    vehicle.center_of_gravity()
  
    print(vehicle.weight_breakdown)
      
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


    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                                            = Segments.Hover.Climb(base_segment)
    segment.tag                                        = "Vertical_Climb"
    segment.analyses.extend(analyses.vertical_climb) 
    segment.altitude_start                             = 0.0  * Units.ft 
    segment.altitude_end                               = 40.  * Units.ft 
    segment.climb_rate                                 = 300. * Units['ft/min'] 
    segment.battery_energy                             = vehicle.networks.battery_propeller.battery.max_energy          
    segment.state.unknowns.throttle                    = 0.8  * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment) 
    mission.append_segment(segment)  
    
 
    # ------------------------------------------------------------------
    #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                      = "Cruise"    
    segment.analyses.extend(analyses.cruise) 
    segment.altitude                 = 2500.0 * Units.ft               
    segment.air_speed                = 175.   * Units['mph']  
    segment.distance                 = 20  * Units.nmi  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)     
    mission.append_segment(segment)     
 
    
  
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
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'vertical_climb'
    vector_angle                                      = 90.0 * Units.degrees
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = prop.inputs.pitch_command_hover
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
    configs.append(config)
   

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------      
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'cruise'
    vector_angle                                      = 0.0 * Units.degrees 
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = prop.inputs.pitch_command_cruise
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
    configs.append(config) 
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

    # Plot Aerodynamic  Forces
    plot_aerodynamic_forces(results)
    
    # Plot Aerodynamic Coefficents
    plot_aerodynamic_coefficients(results)

    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results, line_style)

    # Plot Aircraft Electronics
    plot_battery_pack_conditions(results, line_style)

    # Plot Propeller Conditions 
    plot_propeller_conditions(results, line_style) 

    # Plot Electric Motor and Propeller Efficiencies 
    plot_eMotor_Prop_efficiencies(results, line_style)

    # Plot propeller Disc and Power Loading
    plot_disc_power_loading(results, line_style)   

    # Plot Battery Degradation  
    plot_battery_degradation(results, line_style)       
    
    #if run_noise_model:   
        ## Plot noise level
        #plot_ground_noise_levels(results)
        
        ## Plot noise contour
        #plot_flight_profile_noise_contours(results)  
     
    return     

def save_results(results):

    # Store data (serialize)
    with open('Joby_S4.pkl', 'wb') as file:
        pickle.dump(results, file)
        
    return  

if __name__ == '__main__': 
    main()    
    plt.show()
     
