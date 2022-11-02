# Stopped_Rotor_Vehicle.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data   
import pickle
from SUAVE.Plots.Performance.Mission_Plots import *  
from SUAVE.Plots.Geometry   import * 
from SUAVE.Components.Energy.Networks.Lift_Cruise                            import Lift_Cruise
from SUAVE.Methods.Power.Battery.Sizing                                      import initialize_from_mass 
from SUAVE.Methods.Geometry.Two_Dimensional.Planform                         import segment_properties
from SUAVE.Methods.Power.Battery.Sizing                                      import initialize_from_circuit_configuration 
from SUAVE.Methods.Weights.Correlations.Propulsion                           import nasa_motor
from SUAVE.Methods.Propulsion.electric_motor_sizing                          import size_optimal_motor
from SUAVE.Methods.Propulsion                                                import propeller_design   
from SUAVE.Methods.Weights.Buildups.eVTOL.empty                              import empty
from SUAVE.Methods.Center_of_Gravity.compute_component_centers_of_gravity    import compute_component_centers_of_gravity
from SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_segmented_planform import wing_segmented_planform 
from SUAVE.Methods.Weights.Buildups.eVTOL.converge_evtol_weight              import converge_evtol_weight  

import time 
import os
import numpy as np
import pylab as plt
from copy import deepcopy 

try:
    import vsp 
    from SUAVE.Input_Output.OpenVSP.vsp_write import write 
except ImportError:
    # This allows SUAVE to build without OpenVSP
    pass  

# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def vehicle_setup(MTOW = None):
     
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    vehicle               = SUAVE.Vehicle()
    vehicle.tag           = 'Stopped_Rotor_CRM'
    vehicle.configuration = 'eVTOL'
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    if MTOW == None:
        MTOW = 2900 
    vehicle.mass_properties.max_takeoff       = MTOW
    vehicle.mass_properties.takeoff           = vehicle.mass_properties.max_takeoff
    vehicle.mass_properties.operating_empty   = vehicle.mass_properties.max_takeoff
    vehicle.envelope.ultimate_load            = 5.7   
    vehicle.envelope.limit_load               = 3.  
    vehicle.passengers                        = 6
    
    # ------------------------------------------------------------------    
    # WINGS                                    
    # ------------------------------------------------------------------    
    # WING PROPERTIES           
    wing                          = SUAVE.Components.Wings.Main_Wing()
    wing.tag                      = 'main_wing'  
    wing.aspect_ratio             = 12.27422  # will  be overwritten
    wing.sweeps.quarter_chord     = 0.0  
    wing.thickness_to_chord       = 0.14 
    wing.taper                    = 0.292
    wing.spans.projected          = 14
    wing.chords.root              = 1.75
    wing.total_length             = 1.75
    wing.chords.tip               = 0.525 
    wing.chords.mean_aerodynamic  = 0.8
    wing.dihedral                 = 0.0  
    wing.areas.reference          = 15.  
    wing.twists.root              = 0. * Units.degrees
    wing.twists.tip               = 0. 
    wing.origin                   = [[1.5, 0.,  1.1]]
    wing.aerodynamic_center       = [ 1.567, 0.,  1.1]    
    wing.winglet_fraction         = 0.0  
    wing.symmetric                = True
    wing.vertical                 = False
    
    # Segment                                  
    segment                       = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'Section_1'   
    segment.percent_span_location = 0.  
    segment.twist                 = 4. * Units.degrees 
    segment.root_chord_percent    = 1. 
    segment.dihedral_outboard     = 0 * Units.degrees
    segment.sweeps.quarter_chord  = 3.18 * Units.degrees 
    segment.thickness_to_chord    = 0.16  
    wing.Segments.append(segment)               
    
    # Segment                                   
    segment                       = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'Section_2'    
    segment.percent_span_location = 0.3
    segment.twist                 = 3. * Units.degrees 
    segment.root_chord_percent    = 0.8 # 0.7  
    segment.dihedral_outboard     = 0.0 * Units.degrees
    segment.sweeps.quarter_chord  = 0. 
    segment.thickness_to_chord    = 0.16  
    wing.Segments.append(segment)               
     
    # Segment                                  
    segment                       = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'Section_3'   
    segment.percent_span_location = 1.0
    segment.twist                 = 2.0 * Units.degrees 
    segment.root_chord_percent    = 0.4  # 0.5089086
    segment.dihedral_outboard     = 20   * Units.degrees 
    segment.sweeps.quarter_chord  = 26.45 * Units.degrees 
    segment.thickness_to_chord    = 0.16  
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
    wing                          = SUAVE.Components.Wings.Wing()
    wing.tag                      = 'horizontal_tail'  
    wing.aspect_ratio             = 4.44533 
    wing.sweeps.quarter_chord     = 20. * Units.degrees
    wing.thickness_to_chord       = 0.12
    wing.taper                    = 0.5
    wing.spans.projected          = 2.52
    wing.chords.root              = 0.9494
    wing.total_length             = 0.9494
    wing.chords.tip               = 0.67
    wing.chords.mean_aerodynamic  = 0.809 
    wing.dihedral                 = 35.*Units.degrees
    wing.areas.reference          = 2.915
    wing.areas.wetted             = 2.915 * 2
    wing.areas.exposed            = 2.915 * 2
    wing.twists.root              = 0.0
    wing.twists.tip               = 0.0
    wing.origin                   = [[  5.7 ,0.0 , 0.27]]
    wing.aerodynamic_center       = [  5.7, 0.0, 0.27] 
    wing.winglet_fraction         = 0.0 
    wing.symmetric                = True    
    
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
    segment.percent_z_location                  = 0.04669 
    segment.height                              = 1.409
    segment.width                               = 1.121 
    fuselage.Segments.append(segment)                     
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.651
    segment.percent_z_location                  = 0.03875 
    segment.height                              = 1.11
    segment.width                               = 0.833
    fuselage.Segments.append(segment)                  
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.773
    segment.percent_z_location                  = 0.03612 
    segment.height                              = 0.78
    segment.width                               = 0.512 
    fuselage.Segments.append(segment)                  
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 1.
    segment.percent_z_location                  = 0.03234 
    segment.height                              = 0.195  
    segment.width                               = 0.130 
    fuselage.Segments.append(segment)                   
                                                
    vehicle.append_component(fuselage) 
    
    #-------------------------------------------------------------------
    # INNER BOOMS   
    #-------------------------------------------------------------------   
    boom                                    = SUAVE.Components.Fuselages.Fuselage()
    boom.tag                                = 'boom_1r'
    boom.configuration                      = 'boom'  
    boom.origin                             = [[ 0.227,  1.413,   0.9]]  
    boom.seats_abreast                      = 0.  
    boom.seat_pitch                         = 0.0 
    boom.fineness.nose                      = 0.950   
    boom.fineness.tail                      = 1.029   
    boom.lengths.nose                       = 0.2 
    boom.lengths.tail                       = 0.2
    boom.lengths.cabin                      = 4.15
    boom.lengths.total                      = 4.55
    boom.width                              = 0.15 
    boom.heights.maximum                    = 0.15  
    boom.heights.at_quarter_length          = 0.15  
    boom.heights.at_three_quarters_length   = 0.15 
    boom.heights.at_wing_root_quarter_chord = 0.15 
    boom.areas.wetted                       = 0.018
    boom.areas.front_projected              = 0.018 
    boom.effective_diameter                 = 0.15  
    boom.differential_pressure              = 0.  
    boom.symmetric                          = True 
    boom.index                              = 1
    
    # Segment  
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment() 
    segment.tag                       = 'segment_1'   
    segment.percent_x_location        = 0.
    segment.percent_z_location        = 0.0 
    segment.height                    = 0.05  
    segment.width                     = 0.05   
    boom.Segments.append(segment)           
    
    # Segment                                   
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_2'   
    segment.percent_x_location        = 0.03
    segment.percent_z_location        = 0. 
    segment.height                    = 0.15 
    segment.width                     = 0.15 
    boom.Segments.append(segment) 
    
    # Segment                                   
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_3'    
    segment.percent_x_location        = 0.97
    segment.percent_z_location        = 0. 
    segment.height                    = 0.15
    segment.width                     = 0.15
    boom.Segments.append(segment)           
    
    # Segment                                  
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_4'   
    segment.percent_x_location        = 1.   
    segment.percent_z_location        = 0.   
    segment.height                    = 0.05   
    segment.width                     = 0.05   
    boom.Segments.append(segment)           
    
    # add to vehicle
    vehicle.append_component(boom)   
    
    # add left long boom 
    boom              = deepcopy(vehicle.fuselages.boom_1r)
    boom.origin[0][1] = -boom.origin[0][1]
    boom.tag          = 'boom_1l' 
    vehicle.append_component(boom)         
     
    # add left long boom 
    boom              = deepcopy(vehicle.fuselages.boom_1r)
    boom.origin       = [[ 0.409,  4.022,  1.0]] 
    boom.tag          = 'boom_2r' 
    boom.lengths.total                      = 4.16
    vehicle.append_component(boom)  
    
    # add outer right boom 
    boom              = deepcopy(vehicle.fuselages.boom_1r)
    boom.origin       = [[ 0.409, 6.631,   1.2 ]]    
    boom.lengths.total                      = 4.16
    boom.tag          = 'boom_3r' 
    vehicle.append_component(boom)  
    
    # add inner left boom 
    boom              = deepcopy(vehicle.fuselages.boom_1r)
    boom.origin       = [[ 0.409, -4.022,  1.0 ]]   
    boom.lengths.total                      = 4.16
    boom.tag          = 'boom_2l' 
    vehicle.append_component(boom)     
    
    boom              = deepcopy(vehicle.fuselages.boom_1r)
    boom.origin       =  [[ 0.409, -6.631,  1.2 ]]    
    boom.lengths.total                      = 4.16
    boom.tag          = 'boom_3l' 
    vehicle.append_component(boom)  
    
 
    # ------------------------------------------------------------------
    #   Nacelles
    # ------------------------------------------------------------------ 
    nacelle                = SUAVE.Components.Nacelles.Nacelle()
    nacelle.tag            = 'rotor_nacelle'
    nacelle.length         = 0.45
    nacelle.diameter       = 0.3
    nacelle.orientation_euler_angles  = [0,-90*Units.degrees,0.]    
    nacelle.flow_through   = False  
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_1'
    nac_segment.percent_x_location = 0.0  
    nac_segment.height             = 0.2
    nac_segment.width              = 0.2
    nacelle.append_segment(nac_segment)    
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_2'
    nac_segment.percent_x_location = 0.25  
    nac_segment.height             = 0.25
    nac_segment.width              = 0.25
    nacelle.append_segment(nac_segment)    
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_3'
    nac_segment.percent_x_location = 0.5 
    nac_segment.height             = 0.3
    nac_segment.width              = 0.3
    nacelle.append_segment(nac_segment)    

    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_4'
    nac_segment.percent_x_location = 0.75
    nac_segment.height             = 0.25
    nac_segment.width              = 0.25
    nacelle.append_segment(nac_segment)        

    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 1.0
    nac_segment.height             = 0.2
    nac_segment.width              = 0.2
    nacelle.append_segment(nac_segment)      
 
    lift_rotor_nacelle_origins   = [[ 0.226, 1.413, 0.85] ,[ 0.226, -1.413 , 0.85],
                           [ 4.630 , 1.413 , 0.85] ,[ 4.630 , -1.413 , 0.85],
                           [ 0.409 , 4.022 , 0.95] ,[ 0.409 , -4.022 , 0.95],
                           [ 4.413 , 4.022 , 0.95] ,[ 4.413 , -4.022 , 0.95],
                           [ 0.409 , 6.630 , 1.050] , [0.409 , -6.630 ,1.050],
                           [ 4.413 , 6.630 , 1.050] ,[ 4.413 , -6.630 , 1.050]]
 
    for ii in range(12):
        rotor_nacelle          = deepcopy(nacelle)
        rotor_nacelle.tag      = 'rotor_nacelle_' + str(ii+1) 
        rotor_nacelle.origin   = [lift_rotor_nacelle_origins[ii]]
        vehicle.append_component(rotor_nacelle)   
    
    
    
    nacelle                = SUAVE.Components.Nacelles.Nacelle()
    nacelle.tag            = 'rotor_nacelle'
    nacelle.length         = 1.0
    nacelle.diameter       = 0.4
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
    nac_segment.height             = 0.35
    nac_segment.width              = 0.35
    nacelle.append_segment(nac_segment)    
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_4'
    nac_segment.percent_x_location = 0.5 
    nac_segment.height             = 0.4
    nac_segment.width              = 0.4
    nacelle.append_segment(nac_segment)    

    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 0.75
    nac_segment.height             = 0.35
    nac_segment.width              = 0.35
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

    propeller_nacelle_origins   = [[  6.235, 1.3  ,  1.250] ,[  6.235, -1.3  ,  1.250]]

    for ii in range(2):
        propeller_nacelle          = deepcopy(nacelle)
        propeller_nacelle.tag      = 'propeller_nacelle_' + str(ii+1) 
        propeller_nacelle.origin   = [propeller_nacelle_origins[ii]]
        vehicle.append_component(propeller_nacelle)   
            
    #------------------------------------------------------------------
    # network
    #------------------------------------------------------------------
    net                              = Lift_Cruise()
    net.number_of_lift_rotor_engines = 12
    net.number_of_propeller_engines  = 2
    net.nacelle_diameter             = 0.6 * Units.feet
    net.engine_length                = 0.5 * Units.feet
    net.areas                        = Data()
    net.areas.wetted                 = np.pi*net.nacelle_diameter*net.engine_length + 0.5*np.pi*net.nacelle_diameter**2 
    net.identical_propellers = True
    net.identical_rotors     = True


    #------------------------------------------------------------------
    # Design Electronic Speed Controller
    #------------------------------------------------------------------
    lift_rotor_esc              = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    lift_rotor_esc.efficiency   = 0.95
    net.lift_rotor_esc          = lift_rotor_esc

    propeller_esc            = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    propeller_esc.efficiency = 0.95
    net.propeller_esc        = propeller_esc

    #------------------------------------------------------------------
    # Design Payload
    #------------------------------------------------------------------
    payload                        = SUAVE.Components.Energy.Peripherals.Avionics()
    payload.power_draw             = 10. # Watts 
    payload.mass_properties.mass   = 1.0 * Units.kg
    net.payload                    = payload

    #------------------------------------------------------------------
    # Design Avionics
    #------------------------------------------------------------------
    avionics                       = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw            = 20. # Watts  
    net.avionics                   = avionics  

    #------------------------------------------------------------------
    # Miscellaneous Systems 
    #------------------------------------------------------------------ 
    sys                            = SUAVE.Components.Systems.System()
    sys.mass_properties.mass       = 5 # kg      
    
    #------------------------------------------------------------------
    # Design Battery
    #------------------------------------------------------------------   
    total_cells                         = 150*100
    max_module_voltage                  = 50
    safety_factor                       = 1.5
   
    bat                                 = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650() 
    bat.pack_config.series              = 140  # CHANGE IN OPTIMIZER 
    bat.pack_config.parallel            = int(total_cells/bat.pack_config.series)
    initialize_from_circuit_configuration(bat)    
    net.voltage                         = bat.max_voltage  
    bat.module_config.number_of_modules = 16 # CHANGE IN OPTIMIZER 
    bat.module_config.total             = int(np.ceil(bat.pack_config.total/bat.module_config.number_of_modules))
    bat.module_config.voltage           = net.voltage/bat.module_config.number_of_modules # must be less than max_module_voltage/safety_factor 
    bat.module_config.layout_ratio      = 0.5 # CHANGE IN OPTIMIZER 
    bat.module_config.normal_count      = int(bat.module_config.total**(bat.module_config.layout_ratio))
    bat.module_config.parallel_count    = int(bat.module_config.total**(1-bat.module_config.layout_ratio)) 
    net.battery                         = bat    
    
    #------------------------------------------------------------------
    # Design Rotors and Propellers
    #------------------------------------------------------------------
    # atmosphere and flight conditions for propeller/rotor design  
    g              = 9.81                                   # gravitational acceleration 
    S              = vehicle.reference_area                 # reference area 
    speed_of_sound = 340                                    # speed of sound 
    rho            = 1.22                                   # reference density
    fligth_CL      = 0.75                                   # cruise target lift coefficient 
    AR             = vehicle.wings.main_wing.aspect_ratio   # aspect ratio 
    Cd0            = 0.06                                   # profile drag
    Cdi            = fligth_CL**2/(np.pi*AR*0.98)           # induced drag
    Cd             = Cd0 + Cdi                              # total drag
    V_inf          = 175.* Units['mph']                     # freestream velocity 
    Drag           = S * (0.5*rho*V_inf**2 )*Cd             # cruise drag
    Hover_Load     = vehicle.mass_properties.takeoff*g      # hover load  
    
    # Thrust Propeller        
    propeller                        = SUAVE.Components.Energy.Converters.Propeller()
    propeller.number_of_blades       = 3
    propeller.tag                    = 'propeller_1'
    propeller.freestream_velocity    = V_inf
    propeller.tip_radius             = 1.15
    propeller.hub_radius             = 0.1 * propeller.tip_radius  
    propeller.design_tip_mach        = 0.65
    propeller.angular_velocity       = propeller.design_tip_mach *speed_of_sound/propeller.tip_radius
    propeller.design_Cl              = 0.7
    propeller.design_altitude        = 2500 * Units.feet
    propeller.design_thrust          = 3500 #7000
    propeller.rotation               = 1
    propeller.variable_pitch         = True 
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    airfoil                          = SUAVE.Components.Airfoils.Airfoil()   
    airfoil.coordinate_file          = rel_path + '../Airfoils/NACA_4412.txt'
    airfoil.polar_files              = [rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]
    propeller.append_airfoil(airfoil)    
    propeller.airfoil_polar_stations          = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    propeller                        = propeller_design(propeller)
    
    propeller_origins                = [[   7.126, 1.3  ,  1.250 ] ,[  7.126, -1.3  ,  1.250 ]]
    propeller.origin                 = [propeller_origins[0]]
    net.propellers.append(propeller)  

    propeller_2          = deepcopy(propeller)
    propeller_2.tag      = 'propeller_2'
    propeller_2.rotation = -1
    propeller_2.origin   = [propeller_origins[1]]
    net.propellers.append(propeller_2)  

    # Lift Rotors
    rotor                            = SUAVE.Components.Energy.Converters.Lift_Rotor() 
    rotor.tip_radius                 = 1.15
    rotor.hub_radius                 = 0.1 * rotor.tip_radius  
    rotor.number_of_blades           = 3
    rotor.design_tip_mach            = 0.65   
    rotor.inflow_ratio               = 0.06 
    rotor.angular_velocity           = rotor.design_tip_mach* 343 /rotor.tip_radius   
    rotor.freestream_velocity        = rotor.inflow_ratio*rotor.angular_velocity*rotor.tip_radius 
    rotor.design_Cl                  = 0.7
    rotor.design_altitude            = 20 * Units.feet                     
    rotor.design_thrust              = Hover_Load/(net.number_of_lift_rotor_engines)
    rotor.variable_pitch             = True 

    airfoil                          = SUAVE.Components.Airfoils.Airfoil()   
    airfoil.coordinate_file          = rel_path + '../Airfoils/NACA_4412.txt'
    airfoil.polar_files              = [rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]
    rotor.append_airfoil(airfoil)    
    rotor.airfoil_polar_stations          = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] 
    rotor                            = propeller_design(rotor)


    lift_rotor_origins   = [[ 0.226, 1.413, 1.1] ,[  0.226, -1.413 , 1.1],
                           [ 4.630 , 1.413 , 1.1] ,[ 4.630 , -1.413 , 1.1],
                           [0.409 , 4.022 , 1.2] ,[ 0.409 , -4.022 , 1.2],
                           [ 4.413 , 4.022 , 1.2] ,[ 4.413 , -4.022 , 1.2],
                           [ 0.409 , 6.630 , 1.3] ,[ 0.409 , -6.630 , 1.3],
                           [ 4.413 , 6.630 , 1.3] ,[ 4.413 , -6.630 , 1.3]]
    
    # Appending rotors with different origins
    rotations = [-1,1,-1,1,-1,1,-1,1,-1,1,-1,1] 
    angle_offsets        = np.random.rand(12)*(np.pi)    
    for ii in range(12):
        lift_rotor                        = deepcopy(rotor)
        lift_rotor.tag                    = 'lift_rotor_' + str(ii+1)
        lift_rotor.rotation               = rotations[ii]
        lift_rotor.origin                 = [lift_rotor_origins[ii]]
        lift_rotor.azimuthal_offset_angle = angle_offsets[ii]
        net.lift_rotors.append(lift_rotor)   
    
    #------------------------------------------------------------------
    # Design Motors
    #------------------------------------------------------------------
    # Propeller (Thrust) motor
    propeller_motor                      = SUAVE.Components.Energy.Converters.Motor()
    propeller_motor.efficiency           = 0.80
    propeller_motor.origin               = propeller.origin
    propeller_motor.nominal_voltage      = bat.max_voltage 
    propeller_motor.origin               = propeller.origin
    propeller_motor.propeller_radius     = propeller.tip_radius
    propeller_motor.no_load_current      = 0.01
    propeller_motor                      = size_optimal_motor(propeller_motor,propeller)
    propeller_motor.mass_properties.mass = nasa_motor(propeller_motor.design_torque) 
    net.propeller_motors.append(propeller_motor) 

    propeller_motor_2          = deepcopy(propeller_motor)
    propeller_motor_2.tag      = 'propeller_motor_2' 
    propeller_motor_2.origin   = propeller_2.origin
    net.propeller_motors.append(propeller_motor)
    
        
    # Rotor (Lift) Motor     
    lift_rotor_motor                         = SUAVE.Components.Energy.Converters.Motor()
    lift_rotor_motor.efficiency              = 0.9
    lift_rotor_motor.nominal_voltage         = bat.max_voltage *0.75
    lift_rotor_motor.origin                  = rotor.origin 
    lift_rotor_motor.propeller_radius        = rotor.tip_radius   
    lift_rotor_motor.no_load_current         = 0.01 
    lift_rotor_motor                         = size_optimal_motor(lift_rotor_motor,rotor)
    lift_rotor_motor.mass_properties.mass    = nasa_motor(lift_rotor_motor.design_torque)    

    # Appending motors with different origins
    for _ in range(12):
        motor = deepcopy(lift_rotor_motor)
        motor.tag = 'motor_' + str(ii+1)
        net.lift_rotor_motors.append(motor) 

    # append motor origin spanwise locations onto wing data structure
    vehicle.append_component(net)

    # Add extra drag sources from motors, props, and landing gear. All of these hand measured
    motor_height                     = .25 * Units.feet
    motor_width                      = 1.6 * Units.feet
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
    total_excrescence_area_spin      = 12.*motor_height*motor_width + 2.*main_gear_length*main_gear_width \
        + nose_gear_width*nose_gear_length + 2*main_tire_height*main_tire_width\
        + nose_tire_height*nose_tire_width
    total_excrescence_area_no_spin   = total_excrescence_area_spin + 12*propeller_height*propeller_width
    vehicle.excrescence_area_no_spin = total_excrescence_area_no_spin
    vehicle.excrescence_area_spin    = total_excrescence_area_spin
 
    rotor_motor_origins                   = np.array(lift_rotor_origins)
    propeller_motor_origins               = np.array(propeller_origins) 
    vehicle.wings['main_wing'].motor_spanwise_locations       = rotor_motor_origins[:,1]/vehicle.wings['main_wing'].spans.projected
    vehicle.wings['horizontal_tail'].motor_spanwise_locations = propeller_motor_origins[:,1]/vehicle.wings['horizontal_tail'].spans.projected 
     

    #converge_evtol_weight(vehicle,print_iterations= True ,contingency_factor = 1.0)

    #breakdown = empty(vehicle,contingency_factor = 1.0 )
    #print(breakdown)
    
    #vehicle.weight_breakdown  = breakdown
    #compute_component_centers_of_gravity(vehicle)
    vehicle.center_of_gravity()
    
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
    base_config.networks.lift_cruise.pitch_command = 0
    configs.append(base_config)


    config = SUAVE.Components.Configs.Config(vehicle)
    config.tag = 'transition' 
    configs.append(config)

    config = SUAVE.Components.Configs.Config(vehicle)
    config.tag = 'descent'
    config.networks.lift_cruise.propeller_pitch_command = -5 * Units.degrees
    configs.append(config)

    config = SUAVE.Components.Configs.Config(vehicle)
    config.tag = 'desceleration'
    config.networks.lift_cruise.propeller_pitch_command = -5 * Units.degrees
    configs.append(config)
        
    return configs

