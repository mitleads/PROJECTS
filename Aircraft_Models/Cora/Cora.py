# Stopped_Rotor_CRM.py
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
from SUAVE.Plots.Mission_Plots import *  
from SUAVE.Plots.Geometry_Plots import * 
from SUAVE.Components.Energy.Networks.Lift_Cruise              import Lift_Cruise
from SUAVE.Methods.Weights.Correlations.Propulsion             import nasa_motor
from SUAVE.Methods.Propulsion.electric_motor_sizing            import size_from_mass , size_optimal_motor 
from SUAVE.Methods.Power.Battery.Sizing                        import initialize_from_circuit_configuration    
from SUAVE.Methods.Propulsion                                  import propeller_design   
from SUAVE.Methods.Weights.Buildups.eVTOL.empty import empty

#import vsp 
#from SUAVE.Input_Output.OpenVSP.vsp_write import write

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
    
    breakdown = empty(configs.base,contingency_factor=1.0)
    print(breakdown)     
    
    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate() 

    #save_results(results)

    # plot results
    plot_mission(results)

    return 

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------
def full_setup():  

    # vehicle data
    vehicle  = vehicle_setup()
    #write(vehicle, "Stopped_Rotor_VSP") 
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
    vehicle               = SUAVE.Vehicle()
    vehicle.tag           = 'Stopped_Rotor_V2'
    vehicle.configuration = 'eVTOL'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.takeoff           = 2450. * Units.lb 
    vehicle.mass_properties.operating_empty   = 2250. * Units.lb                
    vehicle.mass_properties.max_takeoff       = 2450. * Units.lb              
    vehicle.mass_properties.center_of_gravity = [[2.0144,   0.  ,  0. ]] # Approximate 
    vehicle.reference_area                    = 10.76  
    vehicle.envelope.ultimate_load            = 5.7   
    vehicle.envelope.limit_load               = 3.  
    vehicle.passengers                        = 2

    # ------------------------------------------------------------------    
    # WINGS                                    
    # ------------------------------------------------------------------    
    # WING PROPERTIES           
    wing                          = SUAVE.Components.Wings.Main_Wing()
    wing.tag                      = 'main_wing'  
    wing.aspect_ratio             = 12.000
    wing.sweeps.quarter_chord     = 0.0  * Units.degrees
    wing.thickness_to_chord       = 0.18  
    wing.taper                    = 1.  
    wing.spans.projected          = 36.0   * Units.feet           # check 
    wing.chords.root              = 6.5    * Units.feet           # check 
    wing.total_length             = 6.5    * Units.feet           # check 
    wing.chords.tip               = 3.     * Units.feet           # check 
    wing.chords.mean_aerodynamic  = 3.     * Units.feet           # check 
    wing.dihedral                 = 1.0    * Units.degrees        # check 
    wing.areas.reference          = 10.582 * Units.meter**2       # check 
    wing.areas.wetted             = 227.5  * Units.feet**2        # check 
    wing.areas.exposed            = 227.5  * Units.feet**2  
    wing.twists.root              = 0.0    * Units.degrees  
    wing.twists.tip               = 0.0    * Units.degrees   
    wing.origin                   = [[  1.067, 0., -0.261 ]]
    wing.aerodynamic_center       = [1.975 , 0., -0.261]    
    wing.winglet_fraction         = 0.0  
    wing.symmetric                = True
    wing.vertical                 = False

    # Segment                                  
    segment                       = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'Section_1'   
    segment.percent_span_location = 0.  
    segment.twist                 = 0.  
    segment.root_chord_percent    = 1. 
    segment.dihedral_outboard     = 2.00 * Units.degrees
    segment.sweeps.quarter_chord  = 30.00 * Units.degrees 
    segment.thickness_to_chord    = 0.18  
    wing.Segments.append(segment)               

    # Segment                                   
    segment                       = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'Section_2'    
    segment.percent_span_location = 0.141 
    segment.twist                 = 0. 
    segment.root_chord_percent    = 0.461383 
    segment.dihedral_outboard     = 2.00 * Units.degrees
    segment.sweeps.quarter_chord  = 0. 
    segment.thickness_to_chord    = 0.16 
    wing.Segments.append(segment)               

    # Segment                                   
    segment                       = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'Section_3'   
    segment.percent_span_location = 0.8478 
    segment.twist                 = 0. 
    segment.root_chord_percent    = 0.461383   
    segment.dihedral_outboard     = 10.00 * Units.degrees 
    segment.sweeps.quarter_chord  = 17.00 * Units.degrees 
    segment.thickness_to_chord    = 0.16 
    wing.Segments.append(segment)               

    # Segment                                  
    segment                       = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'Section_4'   
    segment.percent_span_location = 0.96726 
    segment.twist                 = 0. 
    segment.root_chord_percent    = 0.323 
    segment.dihedral_outboard     = 20.0    * Units.degrees 
    segment.sweeps.quarter_chord  = 51.000  * Units.degrees 
    segment.thickness_to_chord    = 0.16  
    wing.Segments.append(segment)                

    # Segment                                   
    segment                       = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'Section_5'   
    segment.percent_span_location = 1.0 
    segment.twist                 = 0.  
    segment.root_chord_percent    = 0.0413890
    segment.dihedral_outboard     = 1.0  * Units.degrees
    segment.sweeps.quarter_chord  = 0.0 * Units.degrees
    segment.thickness_to_chord    = 0.16 
    wing.Segments.append(segment)   

    # add to vehicle
    vehicle.append_component(wing)       

    # WING PROPERTIES 
    wing                          = SUAVE.Components.Wings.Wing()
    wing.tag                      = 'horizontal_tail'  
    wing.aspect_ratio             = 4.78052
    wing.sweeps.quarter_chord     = 0.0  
    wing.thickness_to_chord       = 0.12  
    wing.taper                    = 1.0  
    wing.spans.projected          = 2.914 
    wing.chords.root              = 0.609
    wing.total_length             = 0.609
    wing.chords.tip               = 0.609
    wing.chords.mean_aerodynamic  = 0.609
    wing.dihedral                 = 0.  * Units.degrees  
    wing.areas.reference          = 5.82204
    wing.areas.wetted             = 5.82204*2 * Units.feet**2    
    wing.areas.exposed            = 5.82204*2 * Units.feet**2  
    wing.twists.root              = 0. * Units.degrees  
    wing.twists.tip               = 0. * Units.degrees  
    wing.origin                   = [[5.440, 0.0 , 1.28]]
    wing.aerodynamic_center       = [5.7,  0.,  0.] 
    wing.winglet_fraction         = 0.0 
    wing.symmetric                = True    

    # add to vehicle
    vehicle.append_component(wing)    


    # WING PROPERTIES
    wing                          = SUAVE.Components.Wings.Wing()
    wing.tag                      = 'vertical_tail_1'
    wing.aspect_ratio             = 4.30556416
    wing.sweeps.quarter_chord     = 13.68 * Units.degrees 
    wing.thickness_to_chord       = 0.12
    wing.taper                    = 0.5 
    wing.spans.projected          = 1.6 #2.578 
    wing.chords.root              = 1.2192
    wing.total_length             = 1.2192
    wing.chords.tip               = 0.6096
    wing.chords.mean_aerodynamic  = 0.9144
    wing.areas.reference          = 2.357
    wing.areas.wetted             = 2.357*2 * Units.feet**2 
    wing.areas.exposed            = 2.357*2 * Units.feet**2 
    wing.twists.root              = 0. * Units.degrees 
    wing.twists.tip               = 0. * Units.degrees  
    wing.origin                   = [[4.900 ,  -1.657 ,  -0.320 ]]
    wing.aerodynamic_center       = 0.0   
    wing.winglet_fraction         = 0.0  
    wing.dihedral                 = 6.  * Units.degrees  
    wing.vertical                 = True 
    wing.symmetric                = False

    # add to vehicle
    vehicle.append_component(wing)   


    # WING PROPERTIES
    wing                         = SUAVE.Components.Wings.Wing()
    wing.tag                     = 'vertical_tail_2'
    wing.aspect_ratio            = 4.30556416
    wing.sweeps.quarter_chord    = 13.68 * Units.degrees 
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.5 
    wing.spans.projected         = 1.6 #2.578 
    wing.chords.root             = 1.2192
    wing.total_length            = 1.2192
    wing.chords.tip              = 0.6096
    wing.chords.mean_aerodynamic = 0.8
    wing.areas.reference         = 2.357
    wing.areas.wetted            = 2.357*2 * Units.feet**2 
    wing.areas.exposed           = 2.357*2 * Units.feet**2 
    wing.twists.root             = 0. * Units.degrees 
    wing.twists.tip              = 0. * Units.degrees  
    wing.origin                  = [[4.900 ,  1.657 ,  -0.320 ]]
    wing.aerodynamic_center      = 0.0   
    wing.winglet_fraction        = 0.0  
    wing.dihedral                = -6.  * Units.degrees  
    wing.vertical                = True   
    wing.symmetric               = False

    # add to vehicle
    vehicle.append_component(wing)   

    # ---------------------------------------------------------------   
    # FUSELAGE                
    # ---------------------------------------------------------------   
    # FUSELAGE PROPERTIES
    fuselage                                    = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage'
    fuselage.configuration                      = 'Tube_Wing'  
    fuselage.seats_abreast                      = 2.  
    fuselage.seat_pitch                         = 3.  
    fuselage.fineness.nose                      = 0.88   
    fuselage.fineness.tail                      = 1.13   
    fuselage.lengths.nose                       = 3.2 * Units.feet 
    fuselage.lengths.tail                       = 6.4 * Units.feet
    fuselage.lengths.cabin                      = 6.4 * Units.feet 
    fuselage.lengths.total                      = 4.10534
    fuselage.width                              = 5.85 * Units.feet        # check 
    fuselage.heights.maximum                    = 4.65 * Units.feet        # check 
    fuselage.heights.at_quarter_length          = 3.75 * Units.feet        # check 
    fuselage.heights.at_wing_root_quarter_chord = 4.65 * Units.feet        # check 
    fuselage.heights.at_three_quarters_length   = 4.26 * Units.feet        # check 
    fuselage.areas.wetted                       = 236. * Units.feet**2     # check 
    fuselage.areas.front_projected              = 0.14 * Units.feet**2     # check 
    fuselage.effective_diameter                 = 5.85 * Units.feet        # check 
    fuselage.differential_pressure              = 0. 

    # Segment  
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment() 
    segment.tag                       = 'segment_0'    
    segment.percent_x_location        = 0.
    segment.percent_z_location        = 0.
    segment.height                    = 0.1  
    segment.width                     = 0.1  
    fuselage.Segments.append(segment)           

    # Segment                                   
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_1'   
    segment.percent_x_location        = 0.04679
    segment.percent_z_location        = 0.00485
    segment.height                    = 0.68505
    segment.width                     = 0.75  
    fuselage.Segments.append(segment) 

    # Segment                                   
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_2'   
    segment.percent_x_location        = 0.17934
    segment.percent_z_location        = 0.01583
    segment.height                    = 1.57170
    segment.width                     = 1.19396
    fuselage.Segments.append(segment)           

    # Segment                                  
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_3'   
    segment.percent_x_location        = 0.47514
    segment.percent_z_location        = 0.02162
    segment.height                    =  1.60249
    segment.width                     =  1.35312
    fuselage.Segments.append(segment)           

    # Segment                                   
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_4'   
    segment.percent_x_location        = 0.73757
    segment.percent_z_location        = 0.05831
    segment.height                    = 0.8379
    segment.width                     = 0.79210 
    fuselage.Segments.append(segment)    

    # Segment                                   
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_5'   
    segment.percent_x_location        =  1.
    segment.percent_z_location        = 0.09127
    segment.height                    = 0.1477
    segment.width                     = 0.0981
    fuselage.Segments.append(segment)           


    # add to vehicle
    vehicle.append_component(fuselage) 

    #-------------------------------------------------------------------
    # INNER BOOMS   
    #-------------------------------------------------------------------   
    long_boom                                    = SUAVE.Components.Fuselages.Fuselage()
    long_boom.tag                                = 'boom_1r'
    long_boom.configuration                      = 'boom'  
    long_boom.origin                             = [[0.543,1.630, -0.326]]  
    long_boom.seats_abreast                      = 0.  
    long_boom.seat_pitch                         = 0.0 
    long_boom.fineness.nose                      = 0.950   
    long_boom.fineness.tail                      = 1.029   
    long_boom.lengths.nose                       = 0.2 
    long_boom.lengths.tail                       = 0.2
    long_boom.lengths.cabin                      = 5.0 
    long_boom.lengths.total                      = 5.4
    long_boom.width                              = 0.15 
    long_boom.heights.maximum                    = 0.15  
    long_boom.heights.at_quarter_length          = 0.15  
    long_boom.heights.at_three_quarters_length   = 0.15 
    long_boom.heights.at_wing_root_quarter_chord = 0.15 
    long_boom.areas.wetted                       = 0.018
    long_boom.areas.front_projected              = 0.018 
    long_boom.effective_diameter                 = 0.15  
    long_boom.differential_pressure              = 0. 
    long_boom.y_pitch_count                      = 1
    long_boom.y_pitch                            = 1.196
    long_boom.symmetric                          = True 
    long_boom.index                              = 1

    # Segment  
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment() 
    segment.tag                       = 'segment_1'   
    segment.percent_x_location        = 0.
    segment.percent_z_location        = 0.0 
    segment.height                    = 0.05  
    segment.width                     = 0.05   
    long_boom.Segments.append(segment)           

    # Segment                                   
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_2'   
    segment.percent_x_location        = 0.2/ 5.6
    segment.percent_z_location        = 0. 
    segment.height                    = 0.15 
    segment.width                     = 0.15 
    long_boom.Segments.append(segment) 

    # Segment                                   
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_3'    
    segment.percent_x_location        = 5.4/5.6 
    segment.percent_z_location        = 0. 
    segment.height                    = 0.15
    segment.width                     = 0.15
    long_boom.Segments.append(segment)           

    # Segment                                  
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_4'   
    segment.percent_x_location        = 1.   
    segment.percent_z_location        = 0.   
    segment.height                    = 0.05   
    segment.width                     = 0.05   
    long_boom.Segments.append(segment)           

    # add to vehicle
    vehicle.append_component(long_boom)   

    # add left long boom 
    long_boom              = deepcopy(vehicle.fuselages.boom_1r)
    long_boom.origin[0][1] = -long_boom.origin[0][1]
    long_boom.tag          = 'Boom_1L'
    long_boom.index        = 1 
    vehicle.append_component(long_boom) 


    #-------------------------------------------------------------------
    # OUTER BOOMS   
    #-------------------------------------------------------------------   
    short_boom                                    = SUAVE.Components.Fuselages.Fuselage()
    short_boom.tag                                = 'boom_2r'
    short_boom.configuration                      = 'boom'  
    short_boom.origin                             = [[0.543,2.826, -0.326]]    
    short_boom.seats_abreast                      = 0.   
    short_boom.seat_pitch                         = 0.0  
    short_boom.fineness.nose                      = 0.950  
    short_boom.fineness.tail                      = 1.029  
    short_boom.lengths.nose                       = 0.2  
    short_boom.lengths.tail                       = 0.2 
    short_boom.lengths.cabin                      = 2.0 
    short_boom.lengths.total                      = 3.3  
    short_boom.width                              = 0.15  
    short_boom.heights.maximum                    = 0.15   
    short_boom.heights.at_quarter_length          = 0.15   
    short_boom.heights.at_three_quarters_length   = 0.15  
    short_boom.heights.at_wing_root_quarter_chord = 0.15  
    short_boom.areas.wetted                       = 0.018 
    short_boom.areas.front_projected              = 0.018  
    short_boom.effective_diameter                 = 0.15   
    short_boom.differential_pressure              = 0. 
    short_boom.y_pitch_count                      = 2
    short_boom.y_pitch                            = 1.196 
    short_boom.symmetric                          = True 
    short_boom.index                              = 1

    # Segment  
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment() 
    segment.tag                       = 'segment_1'   
    segment.percent_x_location        = 0.
    segment.percent_z_location        = 0.0 
    segment.height                    = 0.05  
    segment.width                     = 0.05   
    short_boom.Segments.append(segment)           

    # Segment                                   
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_2'   
    segment.percent_x_location        = 0.2/3.3
    segment.percent_z_location        = 0. 
    segment.height                    = 0.15 
    segment.width                     = 0.15 
    short_boom.Segments.append(segment) 

    # Segment                                   
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_3'   
    segment.percent_x_location        = 3.1/3.3
    segment.percent_z_location        = 0. 
    segment.height                    = 0.15
    segment.width                     = 0.15
    short_boom.Segments.append(segment)           

    # Segment                                  
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_4'     
    segment.percent_x_location        = 1.   
    segment.percent_z_location        = 0.   
    segment.height                    = 0.05   
    segment.width                     = 0.05   
    short_boom.Segments.append(segment)       

    # add to vehicle
    vehicle.append_component(short_boom)    

    # add outer right boom 
    short_boom              = deepcopy(vehicle.fuselages.boom_2r)
    short_boom.origin[0][1] = short_boom.y_pitch + short_boom.origin[0][1]
    short_boom.tag          = 'boom_3r'
    short_boom.index        = 1 
    vehicle.append_component(short_boom)  

    # add inner left boom 
    short_boom              = deepcopy(vehicle.fuselages.boom_2r)
    short_boom.origin[0][1] = - (short_boom.origin[0][1])
    short_boom.tag          = 'boom_2l'
    short_boom.index        = 1 
    vehicle.append_component(short_boom)     

    short_boom              = deepcopy(vehicle.fuselages.boom_2r)
    short_boom.origin[0][1] = - (short_boom.origin[0][1] + short_boom.y_pitch)
    short_boom.tag          = 'boom_3l'
    short_boom.index        = 1 
    vehicle.append_component(short_boom) 

    #------------------------------------------------------------------
    # PROPULSOR
    #------------------------------------------------------------------
    net                             = Lift_Cruise()
    net.number_of_rotor_engines     = 12
    net.number_of_propeller_engines = 1
    net.rotor_thrust_angle          = 90. * Units.degrees
    net.propeller_thrust_angle      = 0.  
    net.rotor_engine_length         = 0.5 * Units.feet
    net.rotor_nacelle_diameter      = 0.6 * Units.feet
    net.propeller_engine_length     = 0.5 * Units.feet
    net.propeller_nacelle_diameter  = 0.6 * Units.feet
    net.propeller_nacelle_end       = 0.8
    net.propeller_nacelle_start     = 0.0   
    net.areas                       = Data()
    net.areas.wetted                = np.pi*net.rotor_nacelle_diameter*net.engine_length + 0.5*np.pi*net.rotor_nacelle_diameter**2  \
                                      + np.pi*net.propeller_nacelle_diameter*net.engine_length + 0.5*np.pi*net.propeller_nacelle_diameter**2 
    net.voltage                     = 500.

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
    bat.pack_config.series         = 120 # 120  
    bat.pack_config.parallel       = 48 # 80    
    initialize_from_circuit_configuration(bat)       
    net.battery                    = bat 
    net.voltage                    = bat.max_voltage    
     

    # Component 6 the Rotor 
    g              = 9.81                                   # gravitational acceleration 
    S              = vehicle.reference_area                 # reference area 
    speed_of_sound = 340                                    # speed of sound 
    rho            = 1.22                                   # reference density
    fligth_CL      = 0.75                                   # cruise target lift coefficient 
    AR             = vehicle.wings.main_wing.aspect_ratio   # aspect ratio 
    Cd0            = 0.06                                   # profile drag
    Cdi            = fligth_CL**2/(np.pi*AR*0.98)           # induced drag
    Cd             = Cd0 + Cdi                              # total drag
    V_inf          = 110.* Units['mph']                     # freestream velocity 
    Drag           = S * (0.5*rho*V_inf**2 )*Cd             # cruise drag
    Hover_Load     = vehicle.mass_properties.takeoff*g      # hover load  

    # Thrust Propeller                          
    propeller                        = SUAVE.Components.Energy.Converters.Propeller() 
    propeller.number_of_blades       = 3
    propeller.number_of_engines      = net.number_of_propeller_engines
    propeller.freestream_velocity    = V_inf
    propeller.tip_radius             = 1.0668
    propeller.hub_radius             = 0.21336 
    propeller.design_tip_mach        = 0.5  
    propeller.angular_velocity       = propeller.design_tip_mach *speed_of_sound/propeller.tip_radius   
    propeller.design_Cl              = 0.7
    propeller.design_altitude        = 1000 * Units.feet   
    propeller.design_thrust          = (Drag*2.5)/net.number_of_propeller_engines  
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    propeller.airfoil_geometry       =  [rel_path +'NACA_4412.txt']
    propeller.airfoil_polars         = [[rel_path +'NACA_4412_polar_Re_50000.txt' ,
                                         rel_path +'NACA_4412_polar_Re_100000.txt' ,
                                         rel_path +'NACA_4412_polar_Re_200000.txt' ,
                                         rel_path +'NACA_4412_polar_Re_500000.txt' ,
                                         rel_path +'NACA_4412_polar_Re_1000000.txt' ]]
    propeller.airfoil_polar_stations = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]      
    propeller                        = propeller_design(propeller)   
    propeller.origin                 = [[4.10534, 0. ,2.02*0.3048 ]]   
    net.propeller                    = propeller

    # Lift Rotors                                  
    rotor                            = SUAVE.Components.Energy.Converters.Rotor() 
    rotor.tip_radius                 = 2.8 * Units.feet
    rotor.hub_radius                 = 0.35 * Units.feet      
    rotor.number_of_blades           = 2
    rotor.design_tip_mach            = 0.65
    rotor.number_of_engines          = net.number_of_rotor_engines
    rotor.disc_area                  = np.pi*(rotor.tip_radius**2)        
    rotor.disc_loading               = (Hover_Load/Units.lb)/( rotor.disc_area/Units.feet**2* net.number_of_rotor_engines)
    rotor.induced_hover_velocity     = np.sqrt(Hover_Load/(2*rho*rotor.disc_area*net.number_of_rotor_engines)) 
    rotor.freestream_velocity        = 500. * Units['ft/min']  
    rotor.angular_velocity           = rotor.design_tip_mach* speed_of_sound /rotor.tip_radius   
    rotor.design_Cl                  = 0.7
    rotor.design_altitude            = 20 * Units.feet                            
    rotor.design_thrust              = Hover_Load/(net.number_of_rotor_engines-1) # 2*Hover_Load/(net.number_of_rotor_engines) 
    rotor.x_pitch_count              = 2
    rotor.y_pitch_count              = vehicle.fuselages['boom_1r'].y_pitch_count
    rotor.y_pitch                    = vehicle.fuselages['boom_1r'].y_pitch 
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    rotor.airfoil_geometry           =  [rel_path +'NACA_4412.txt']
    rotor.airfoil_polars             = [[rel_path +'NACA_4412_polar_Re_50000.txt' ,
                                         rel_path +'NACA_4412_polar_Re_100000.txt' ,
                                         rel_path +'NACA_4412_polar_Re_200000.txt' ,
                                         rel_path +'NACA_4412_polar_Re_500000.txt' ,
                                         rel_path +'NACA_4412_polar_Re_1000000.txt' ]]
    rotor.airfoil_polar_stations     = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]       
    rotor                            = propeller_design(rotor)          
    rotor.rotation                   = [1,1,1,1,1,1,1,1,1,1,1,1] 
    rotor.origin                     = [[0.543,  1.63  , 0] ,[0.543, -1.63  , 0 ] ,
                                        [3.843,  1.63  , 0] ,[3.843, -1.63  , 0 ] ,
                                        [0.543,  2.826 , 0] ,[0.543, -2.826 , 0 ] ,
                                        [3.843,  2.826 , 0] ,[3.843, -2.826 , 0 ] ,
                                        [0.543,  4.022 , 0] ,[0.543, -4.022 , 0 ] ,
                                        [3.843,  4.022 , 0] ,[3.843, -4.022 , 0 ]]
    rotor.symmetric                  = True
    net.number_of_rotor_engines      = 12

    # append propellers to vehicle     
    net.rotor = rotor

    #------------------------------------------------------------------
    # Design Motors
    #------------------------------------------------------------------
    # Propeller (Thrust) motor
    propeller_motor                      = SUAVE.Components.Energy.Converters.Motor()
    propeller_motor.efficiency           = 0.95
    propeller_motor.nominal_voltage      = bat.max_voltage  
    propeller_motor.origin               = propeller.origin  
    propeller_motor.propeller_radius     = propeller.tip_radius      
    propeller_motor.no_load_current      = 0.1
    propeller_motor                      = size_optimal_motor(propeller_motor,propeller)
    propeller_motor.mass_properties.mass = nasa_motor(propeller_motor.design_torque)
    net.propeller_motor                  = propeller_motor

    # Rotor (Lift) Motor     
    rotor_motor                         = SUAVE.Components.Energy.Converters.Motor()
    rotor_motor.efficiency              = 0.95  
    rotor_motor.nominal_voltage         = bat.max_voltage  * 3/4
    rotor_motor.origin                  = rotor.origin  
    rotor_motor.propeller_radius        = rotor.tip_radius   
    rotor_motor.no_load_current         = 0.01 
    rotor_motor                         = size_optimal_motor(rotor_motor,rotor)
    rotor_motor.mass_properties.mass    = nasa_motor(rotor_motor.design_torque)
    net.rotor_motor                     = rotor_motor  

    # append motor origin spanwise locations onto wing data structure 
    vehicle.append_component(net) 

    # Add extra drag sources from motors, props, and landing gear. All of these hand measured 
    motor_height                        = .25 * Units.feet
    motor_width                         = 1.6 * Units.feet    
    propeller_width                     = 1. * Units.inches
    propeller_height                    = propeller_width *.12    
    main_gear_width                     = 1.5 * Units.inches
    main_gear_length                    = 2.5 * Units.feet    
    nose_gear_width                     = 2. * Units.inches
    nose_gear_length                    = 2. * Units.feet    
    nose_tire_height                    = (0.7 + 0.4) * Units.feet
    nose_tire_width                     = 0.4 * Units.feet    
    main_tire_height                    = (0.75 + 0.5) * Units.feet
    main_tire_width                     = 4. * Units.inches    
    total_excrescence_area_spin         = 12.*motor_height*motor_width + 2.*main_gear_length*main_gear_width \
        + nose_gear_width*nose_gear_length + 2*main_tire_height*main_tire_width\
                                              + nose_tire_height*nose_tire_width 
    total_excrescence_area_no_spin      = total_excrescence_area_spin + 12*propeller_height*propeller_width  
    vehicle.excrescence_area_no_spin    = total_excrescence_area_no_spin 
    vehicle.excrescence_area_spin       = total_excrescence_area_spin 

    rotor_motor_origins                  = np.concatenate([rotor.origin,propeller.origin]) 
    vehicle.wings['main_wing'].motor_spanwise_locations = rotor_motor_origins[:,1]/wing.spans.projected
    vehicle.wings['horizontal_tail'].motor_spanwise_locations = np.array([0.])
    vehicle.wings['vertical_tail_1'].motor_spanwise_locations = np.array([0.])
    vehicle.wings['vertical_tail_2'].motor_spanwise_locations = np.array([0.])

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

    # ------------------------------------------------------------------
    #  Noise Analysis
    noise = SUAVE.Analyses.Noise.Fidelity_One()   
    noise.geometry = vehicle
    analyses.append(noise)

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

    return configs

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
    base_segment.process.iterate.unknowns.network            = vehicle.propulsors.lift_cruise.unpack_unknowns_transition
    base_segment.process.iterate.residuals.network           = vehicle.propulsors.lift_cruise.residuals_transition
    base_segment.state.unknowns.battery_voltage_under_load   = vehicle.propulsors.lift_cruise.battery.max_voltage * ones_row(1)  
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
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment     = Segments.Hover.Climb(base_segment)
    segment.tag = "climb_1" 

    segment.analyses.extend( analyses.base ) 

    segment.altitude_start                                   = 0.0  * Units.ft
    segment.altitude_end                                     = 40.  * Units.ft
    segment.climb_rate                                       = 500. * Units['ft/min']
    segment.battery_energy                                   = vehicle.propulsors.lift_cruise.battery.max_energy

    segment.state.unknowns.rotor_power_coefficient           = 0.016 * ones_row(1) 
    segment.state.unknowns.throttle_lift                     = 0.9   * ones_row(1) 
    segment.state.unknowns.__delitem__('throttle')

    segment.process.iterate.unknowns.network                 = vehicle.propulsors.lift_cruise.unpack_unknowns_no_forward
    segment.process.iterate.residuals.network                = vehicle.propulsors.lift_cruise.residuals_no_forward       
    segment.process.iterate.unknowns.mission                 = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability             = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability          = SUAVE.Methods.skip

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Cruise Segment: Transition
    # ------------------------------------------------------------------

    segment     = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
    segment.tag = "transition_1"

    segment.analyses.extend( analyses.base ) 
    segment.altitude        = 40.  * Units.ft
    segment.air_speed_start = 500. * Units['ft/min']
    segment.air_speed_end   = 0.8 * Vstall
    segment.acceleration    = 9.8/5
    segment.pitch_initial   = 0.0 * Units.degrees
    segment.pitch_final     = 5. * Units.degrees

    segment.state.unknowns.rotor_power_coefficient          = 0.05 *  ones_row(1)  
    segment.state.unknowns.throttle_lift                    = 0.9  * ones_row(1)   
    segment.state.unknowns.propeller_power_coefficient      = 0.14 *  ones_row(1) 
    segment.state.unknowns.throttle                         = 0.95  *  ones_row(1) 
    segment.state.residuals.network                         = 0.   *  ones_row(3) 

    segment.process.iterate.unknowns.network                = vehicle.propulsors.lift_cruise.unpack_unknowns_transition
    segment.process.iterate.residuals.network               = vehicle.propulsors.lift_cruise.residuals_transition    
    segment.process.iterate.unknowns.mission                = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability         = SUAVE.Methods.skip 

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Cruise Segment: Transition
    # ------------------------------------------------------------------

    segment     = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
    segment.tag = "transition_2"

    segment.analyses.extend( analyses.base ) 
    segment.altitude_start         = 40.0 * Units.ft
    segment.altitude_end           = 100.0 * Units.ft
    segment.air_speed              = 0.8 * Vstall
    segment.climb_angle            = 1 * Units.degrees
    segment.acceleration           = 0.5 * Units['m/s/s']    
    segment.pitch_initial          = 5. * Units.degrees  
    segment.pitch_final            = 7. * Units.degrees       

    segment.state.unknowns.rotor_power_coefficient          = 0.02  * ones_row(1)
    segment.state.unknowns.throttle_lift                    = 0.8  * ones_row(1) 
    segment.state.unknowns.propeller_power_coefficient      = 0.16  * ones_row(1)
    segment.state.unknowns.throttle                         = 0.80  * ones_row(1)   
    segment.state.residuals.network                         = 0.    * ones_row(3)    

    segment.process.iterate.unknowns.network                = vehicle.propulsors.lift_cruise.unpack_unknowns_transition
    segment.process.iterate.residuals.network               = vehicle.propulsors.lift_cruise.residuals_transition    
    segment.process.iterate.unknowns.mission                = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability         = SUAVE.Methods.skip

    # add to misison
    mission.append_segment(segment) 

    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment     = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2"

    segment.analyses.extend( analyses.base )

    segment.air_speed       = 1.2  * Vstall
    segment.altitude_start  = 100.0 * Units.ft
    segment.altitude_end    = 300. * Units.ft
    segment.climb_rate      = 500. * Units['ft/min']

    segment.state.unknowns.propeller_power_coefficient         = 0.16   * ones_row(1)
    segment.state.unknowns.throttle                            = 0.80   * ones_row(1)
    segment.process.iterate.unknowns.network  = vehicle.propulsors.lift_cruise.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.lift_cruise.residuals_no_lift     

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------

    segment     = Segments.Cruise.Constant_Speed_Constant_Altitude_Loiter(base_segment)
    segment.tag = "departure_terminal_procedures"

    segment.analyses.extend( analyses.base )

    segment.altitude  = 300.0 * Units.ft
    segment.time      = 60.   * Units.second
    segment.air_speed = 1.2*Vstall

    segment.state.unknowns.propeller_power_coefficient =  0.16   * ones_row(1)
    segment.state.unknowns.throttle                    =  0.80   * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.lift_cruise.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.lift_cruise.residuals_no_lift     


    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Climb Segment: Constant Acceleration, Constant Rate
    # ------------------------------------------------------------------

    segment     = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "accelerated_climb"

    segment.analyses.extend( analyses.base )

    segment.altitude_start  = 300.0 * Units.ft
    segment.altitude_end    = 2500. * Units.ft
    segment.climb_rate      = 500.  * Units['ft/min']
    segment.air_speed_start = 1.2   * Vstall
    segment.air_speed_end   = 110.  * Units['mph']                                            

    segment.state.unknowns.propeller_power_coefficient =  0.16   * ones_row(1)
    segment.state.unknowns.throttle                    =  0.80   * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.lift_cruise.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.lift_cruise.residuals_no_lift   

    # add to misison
    mission.append_segment(segment)    

    # ------------------------------------------------------------------
    #   Third Cruise Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------

    segment     = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.base )

    segment.altitude  = 2500.0 * Units.ft
    segment.air_speed = 110.   * Units['mph']
    segment.distance  = 60.    * Units.miles                       

    segment.state.unknowns.propeller_power_coefficient = 0.16 * ones_row(1)  
    segment.state.unknowns.throttle                    = 0.80 * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.lift_cruise.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.lift_cruise.residuals_no_lift    


    # add to misison
    mission.append_segment(segment)     


    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Acceleration, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "decelerating_descent"

    segment.analyses.extend( analyses.base )  
    segment.altitude_start  = 2500.0 * Units.ft
    segment.altitude_end    = 300. * Units.ft
    segment.climb_rate      = -300.  * Units['ft/min']
    segment.air_speed_start = 110.  * Units['mph']
    segment.air_speed_end   = 1.2*Vstall

    segment.state.unknowns.propeller_power_coefficient =  0.2 * ones_row(1)
    segment.state.unknowns.throttle                    =  0.8 * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.lift_cruise.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.lift_cruise.residuals_no_lift     

    # add to misison
    mission.append_segment(segment)        

    # ------------------------------------------------------------------
    #   Reserve Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude_Loiter(base_segment)
    segment.tag = "arrival_terminal_procedures"

    segment.analyses.extend( analyses.base )

    segment.altitude        = 300.   * Units.ft
    segment.air_speed       = 1.2*Vstall
    segment.time            = 60 * Units.seconds

    segment.state.unknowns.propeller_power_coefficient = 0.2 * ones_row(1)
    segment.state.unknowns.throttle                    = 0.8 * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.lift_cruise.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.lift_cruise.residuals_no_lift   

    # add to misison
    mission.append_segment(segment)    


    # ------------------------------------------------------------------
    #   Reserve Segment: Constant Acceleration, Constant Rate
    # ------------------------------------------------------------------

    segment     = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "reserve_accelerated_climb"

    segment.analyses.extend( analyses.base )

    segment.altitude_start  = 300.0 * Units.ft
    segment.altitude_end    = 1500. * Units.ft
    segment.climb_rate      = 500.  * Units['ft/min']
    segment.air_speed_start = 1.2   * Vstall
    segment.air_speed_end   = 110.  * Units['mph']                                            

    segment.state.unknowns.propeller_power_coefficient =  0.16   * ones_row(1)
    segment.state.unknowns.throttle                    =  0.80   * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.lift_cruise.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.lift_cruise.residuals_no_lift   

    # add to misison
    mission.append_segment(segment)    

    # ------------------------------------------------------------------
    #   Reserve Cruise Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------

    segment     = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "reserve_cruise"

    segment.analyses.extend( analyses.base )

    segment.altitude  = 1500.0 * Units.ft
    segment.air_speed = 110.   * Units['mph']
    segment.distance  = 33.5    * Units.miles  * 0.1                          

    segment.state.unknowns.propeller_power_coefficient = 0.16 * ones_row(1)  
    segment.state.unknowns.throttle                    = 0.80 * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.lift_cruise.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.lift_cruise.residuals_no_lift    


    # add to misison
    mission.append_segment(segment)     


    # ------------------------------------------------------------------
    #   Reserve Descent Segment: Constant Acceleration, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "reserve_decelerating_descent"

    segment.analyses.extend( analyses.base )  
    segment.altitude_start  = 1500.0 * Units.ft
    segment.altitude_end    = 300. * Units.ft
    segment.climb_rate      = -300.  * Units['ft/min']
    segment.air_speed_start = 110.  * Units['mph']
    segment.air_speed_end   = 1.2*Vstall

    segment.state.unknowns.propeller_power_coefficient =  0.2 * ones_row(1)
    segment.state.unknowns.throttle                    =  0.8 * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.lift_cruise.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.lift_cruise.residuals_no_lift     

    # add to misison
    mission.append_segment(segment)        

    # ------------------------------------------------------------------
    #  Reserve Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude_Loiter(base_segment)
    segment.tag = "reserve_arrival_terminal_procedures"  

    segment.analyses.extend( analyses.base )

    segment.altitude        = 300.    * Units.ft
    segment.air_speed       = 1.2     * Vstall
    segment.time            = 60      * Units.seconds

    segment.state.unknowns.propeller_power_coefficient = 0.12 * ones_row(1)
    segment.state.unknowns.throttle                    = 0.75 * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.lift_cruise.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.lift_cruise.residuals_no_lift   

    # add to misison
    mission.append_segment(segment)        


    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_2"

    segment.analyses.extend( analyses.base )

    segment.altitude_start  = 300.0 * Units.ft
    segment.altitude_end    = 90. * Units.ft
    segment.climb_rate      = -300.  * Units['ft/min']  
    segment.air_speed_start = 1.2*Vstall
    segment.air_speed_end   = 1.1*Vstall    
    segment.state.unknowns.propeller_power_coefficient = 0.2 * ones_row(1)
    segment.state.unknowns.throttle                    = 0.8 * ones_row(1)

    segment.process.iterate.unknowns.network  = vehicle.propulsors.lift_cruise.unpack_unknowns_no_lift
    segment.process.iterate.residuals.network = vehicle.propulsors.lift_cruise.residuals_no_lift 


    # add to misison
    mission.append_segment(segment)       

    # ------------------------------------------------------------------
    #   First Cruise Segment: Transition
    # ------------------------------------------------------------------

    segment     = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
    segment.tag = "final_transition"

    segment.analyses.extend( analyses.base )

    segment.altitude_start         = 90.0 * Units.ft
    segment.altitude_end           = 40.0 * Units.ft
    segment.air_speed              = 1.1 * Vstall
    segment.climb_angle            = 1 * Units.degrees
    segment.acceleration           = -0.2 * Units['m/s/s']    
    segment.pitch_initial          = 9. * Units.degrees  
    segment.pitch_final            = 12. * Units.degrees       

    segment.state.unknowns.rotor_power_coefficient          = 0.04 *  ones_row(1)  
    segment.state.unknowns.throttle_lift                    = 0.4  * ones_row(1)   
    segment.state.unknowns.propeller_power_coefficient      = 0.15 *  ones_row(1) 
    segment.state.unknowns.throttle                         = 0.65 *  ones_row(1) 
    segment.state.residuals.network                         = 0.   *  ones_row(3)   

    segment.process.iterate.unknowns.network                = vehicle.propulsors.lift_cruise.unpack_unknowns_transition
    segment.process.iterate.residuals.network               = vehicle.propulsors.lift_cruise.residuals_transition    
    segment.process.iterate.unknowns.mission                = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability            = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability         = SUAVE.Methods.skip

    # add to misison
    mission.append_segment(segment) 

    # ------------------------------------------------------------------
    #   Fifth Cuise Segment: Transition
    # ------------------------------------------------------------------ 
    segment = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
    segment.tag = "descent_transition"

    segment.analyses.extend( analyses.base )

    segment.altitude        = 40.  * Units.ft
    segment.air_speed_start = 0.8  * Vstall      
    segment.air_speed_end   = 300. * Units['ft/min'] 
    segment.acceleration    = -0.5
    segment.pitch_initial   = 12. * Units.degrees   
    segment.pitch_final     = 10. * Units.degrees      

    segment.state.unknowns.rotor_power_coefficient          = 0.04 *  ones_row(1) 
    segment.state.unknowns.throttle_lift                    = 0.85 *  ones_row(1) 
    segment.state.unknowns.propeller_power_coefficient      = 0.01 *  ones_row(1)   
    segment.state.unknowns.throttle                         = 0.5  *  ones_row(1)   
    segment.state.residuals.network                         = 0.   * ones_row(3)    

    segment.process.iterate.unknowns.network  = vehicle.propulsors.lift_cruise.unpack_unknowns_transition
    segment.process.iterate.residuals.network = vehicle.propulsors.lift_cruise.residuals_transition    
    segment.process.iterate.unknowns.mission  = SUAVE.Methods.skip 

    # add to misison
    mission.append_segment(segment)  

    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Hover.Descent(base_segment)
    segment.tag = "descent_1"

    segment.analyses.extend( analyses.base )

    segment.altitude_start  = 40.0  * Units.ft
    segment.altitude_end    = 0.   * Units.ft
    segment.descent_rate    = 300. * Units['ft/min']  

    segment.state.unknowns.rotor_power_coefficient           = 0.04 * ones_row(1) 
    segment.state.unknowns.throttle_lift                     = 0.85  * ones_row(1) 

    segment.state.unknowns.__delitem__('throttle')
    segment.process.iterate.unknowns.network  = vehicle.propulsors.lift_cruise.unpack_unknowns_no_forward
    segment.process.iterate.residuals.network = vehicle.propulsors.lift_cruise.residuals_no_forward    
    segment.process.iterate.unknowns.mission  = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability      = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability   = SUAVE.Methods.skip

    # add to misison
    mission.append_segment(segment)       

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