# Stopped_Rotor_Analyses.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE 
from SUAVE.Plots.Performance.Mission_Plots import *  
from SUAVE.Plots.Geometry   import *

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ---------------------------------------------------------------------- 
def topography_analyses_setup(configs,topography_data,run_noise_model):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = topography_base_analysis(config,topography_data,run_noise_model)
        analyses[tag] = analysis

    return analyses


# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ---------------------------------------------------------------------- 
def level_ground_analyses_setup(configs,level_ground_data,run_noise_model):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = level_ground_base_analysis(config,level_ground_data,run_noise_model)
        analyses[tag] = analysis

    return analyses
 

# ------------------------------------------------------------------
# Base Analysis
# ------------------------------------------------------------------
def topography_base_analysis(vehicle,topography_data,run_noise_model):

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
    aerodynamics.settings.model_fuselage = True 
    aerodynamics.settings.number_spanwise_vortices           = 25
    aerodynamics.settings.number_chordwise_vortices          = 5    
    analyses.append(aerodynamics)  
        
    if run_noise_model:  
        # ------------------------------------------------------------------
        #  Noise Analysis
        noise = SUAVE.Analyses.Noise.Fidelity_One()   
        noise.geometry = vehicle
        noise.settings.ground_microphone_x_resolution   = topography_data.ground_microphone_x_resolution           
        noise.settings.ground_microphone_y_resolution   = topography_data.ground_microphone_y_resolution          
        noise.settings.ground_microphone_x_stencil      = topography_data.ground_microphone_x_stencil             
        noise.settings.ground_microphone_y_stencil      = topography_data.ground_microphone_y_stencil             
        noise.settings.ground_microphone_min_y          = topography_data.ground_microphone_min_x                 
        noise.settings.ground_microphone_max_y          = topography_data.ground_microphone_max_x                 
        noise.settings.ground_microphone_min_x          = topography_data.ground_microphone_min_y                 
        noise.settings.ground_microphone_max_x          = topography_data.ground_microphone_max_y                 
        noise.settings.ground_microphone_locations      = topography_data.cartesian_microphone_locations            
        noise.settings.aircraft_departure_location      = topography_data.departure_location   
        noise.settings.aircraft_destimation_location    = topography_data.destination_location                              
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
 
# ------------------------------------------------------------------
# Base Analysis
# ------------------------------------------------------------------
def level_ground_base_analysis(vehicle,level_ground_data,run_noise_model):

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
    aerodynamics.settings.model_fuselage = True 
    aerodynamics.settings.number_spanwise_vortices           = 25
    aerodynamics.settings.number_chordwise_vortices          = 5    
    analyses.append(aerodynamics)  
        
    if run_noise_model:  
        # ------------------------------------------------------------------
        #  Noise Analysis
        noise = SUAVE.Analyses.Noise.Fidelity_One()   
        noise.geometry = vehicle                     
        noise.settings.ground_microphone_x_resolution   = level_ground_data.ground_microphone_x_resolution           
        noise.settings.ground_microphone_y_resolution   = level_ground_data.ground_microphone_y_resolution          
        noise.settings.ground_microphone_x_stencil      = level_ground_data.ground_microphone_x_stencil             
        noise.settings.ground_microphone_y_stencil      = level_ground_data.ground_microphone_y_stencil             
        noise.settings.ground_microphone_min_y          = level_ground_data.ground_microphone_min_x                 
        noise.settings.ground_microphone_max_y          = level_ground_data.ground_microphone_max_x                 
        noise.settings.ground_microphone_min_x          = level_ground_data.ground_microphone_min_y                 
        noise.settings.ground_microphone_max_x          = level_ground_data.ground_microphone_max_y                 
        noise.settings.ground_microphone_locations      = level_ground_data.cartesian_microphone_locations         
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
