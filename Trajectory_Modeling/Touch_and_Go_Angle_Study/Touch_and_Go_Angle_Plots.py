# Vehicle Comparisons

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------    
from SUAVE.Plots.Performance.Mission_Plots import *
from SUAVE.Plots.Geometry import *  
from SUAVE.Core import Data , Units
import matplotlib.gridspec as gridspec
import scipy.ndimage as ndimage
import numpy as np    
import matplotlib.pyplot as plt    
import numpy as np   
import pickle  
 
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main(): 
    plt.rcParams['axes.linewidth'] = 1.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 24,
                  'xtick.labelsize': 22,
                  'ytick.labelsize': 22,
                  'axes.titlesize': 22}
    plt.rcParams.update(parameters)
    plot_parameters                  = Data()
    plot_parameters.line_width       = 3
    plot_parameters.line_style       = ['-','--']
    plot_parameters.figure_width     = 12 
    plot_parameters.figure_height    = 10 
    plot_parameters.marker_size      = 10 
    plot_parameters.legend_font_size = 16 
    plot_parameters.plot_grid        = True   
    plot_parameters.markers          = ['o','^','X','s','P','D','X','*']
    plot_parameters.colors           = ['black','mediumblue','darkgreen','firebrick']   
    plot_parameters.colors2          = ['grey','darkcyan','green','red']                  
    plot_parameters.legend_font      = 16                          
    plot_parameters.marker_size      = 14   
     
     
    N_gm_x                 = 21  
    N_gm_y                 = 21  
    header                 = 'Results/'  
    vehicle_tag            = 'SR_V2_TG_Noise_Angle_' 
    true_course_angles_deg = [0,45,90,135,180] 
    
    for ang in range(len(true_course_angles_deg)):
        true_course_angle_deg = true_course_angles_deg[ang] 
        sr_noise_filename      = header + vehicle_tag + str(true_course_angle_deg) + '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y) 
        sr_noise_results_raw   = load_results(sr_noise_filename )    
        plot_noise_contour(sr_noise_results_raw,plot_parameters,save_fig_name =  'SR_'+ str(true_course_angle_deg) ) 
         
    return
 
# ------------------------------------------------------------------
# Process Results 
# ------------------------------------------------------------------
def process_results(res,N_gm_x ,N_gm_y,vehicle_name ):
    '''This function cleans up the data and connects segments for plots ''' 

    num_ctrl_pts   = len(res.segments[0].conditions.frames.inertial.time[:,0] )
    num_segments   = len(res.segments.keys()) 
    data_dimension = num_ctrl_pts*num_segments
    
    PD                 = Data()
    PD.time            = np.zeros(data_dimension)
    PD.ground_distance = np.zeros(data_dimension)
    PD.altitude        = np.zeros(data_dimension) 
    PD.time_normalized = np.zeros(data_dimension)   
    PD.num_gm          = res.segments[0].conditions.noise.number_of_ground_microphones 
    PD.gm_mic_loc      = res.segments[0].analyses.noise.settings.ground_microphone_locations.reshape(N_gm_x,N_gm_y,3)
    PD.SPL_contour     = np.zeros((data_dimension,N_gm_x,N_gm_y)) 
    PD.N_gm_x          = N_gm_x  
    PD.N_gm_y          = N_gm_y  
    PD.aircraft_pos    = np.zeros((data_dimension,3))  
    PD.num_segments    = len(res.segments)    
    PD.num_ctrl_pts    = num_ctrl_pts
    
    Flight_Time        = res.segments[-1].conditions.frames.inertial.time[-1,0]
    for i in range(num_segments):  
        time            = res.segments[i].conditions.frames.inertial.time[:,0]  
        time_normalized = res.segments[i].conditions.frames.inertial.time[:,0]/Flight_Time
        ground_distance = res.segments[i].conditions.frames.inertial.position_vector[:,0]  
        altitude        = res.segments[i].conditions.freestream.altitude[:,0]  
        SPL_contour     = res.segments[i].conditions.noise.total_SPL_dBA   
        pos             = res.segments[i].conditions.frames.inertial.position_vector  
        
        SPL_contour = np.nan_to_num(SPL_contour)
        print(np.max(SPL_contour))
        
        PD.time_normalized[i*num_ctrl_pts:(i+1)*num_ctrl_pts] = time_normalized
        PD.time[i*num_ctrl_pts:(i+1)*num_ctrl_pts]            = time          
        PD.ground_distance[i*num_ctrl_pts:(i+1)*num_ctrl_pts] = ground_distance      
        PD.altitude[i*num_ctrl_pts:(i+1)*num_ctrl_pts]        = altitude       
        PD.SPL_contour[i*num_ctrl_pts:(i+1)*num_ctrl_pts,:,:] = SPL_contour.reshape(num_ctrl_pts,N_gm_x,N_gm_y)   
        PD.aircraft_pos[i*num_ctrl_pts:(i+1)*num_ctrl_pts,:]  = pos       
        
            
    return PD


def post_process_noise_data(results): 

    # unpack 
    background_noise_dbA = 35 
    N_segs               = len(results.segments)
    N_ctrl_pts           = len(results.segments[0].conditions.frames.inertial.time[:,0]) 
    N_bm                 = results.segments[0].conditions.noise.number_of_building_microphones 
    N_gm_x               = results.segments[0].analyses.noise.settings.microphone_x_resolution
    N_gm_y               = results.segments[0].analyses.noise.settings.microphone_y_resolution   
    dim_mat              = N_segs*N_ctrl_pts 
    SPL_contour_gm       = np.ones((dim_mat,N_gm_x,N_gm_y))*background_noise_dbA
    SPL_contour_bm       = np.ones((dim_mat,N_bm))*background_noise_dbA
    Aircraft_pos         = np.zeros((dim_mat,3)) 
    Mic_pos_gm           = results.segments[0].conditions.noise.total_ground_microphone_locations[0].reshape(N_gm_x,N_gm_y,3) 
    
    for i in range(N_segs):  
        if  results.segments[i].battery_discharge == False:
            pass
        else:      
            S_gm_x = results.segments[i].analyses.noise.settings.microphone_x_stencil
            S_gm_y = results.segments[i].analyses.noise.settings.microphone_y_stencil
            S_locs = results.segments[i].conditions.noise.ground_microphone_stencil_locations
            for j in range(N_ctrl_pts):
                idx                    = i*N_ctrl_pts + j 
                Aircraft_pos[idx,0]    = results.segments[i].conditions.frames.inertial.position_vector[j,0] 
                Aircraft_pos[idx,1]    = results.segments[i].conditions.frames.inertial.position_vector[j,1] 
                Aircraft_pos[idx,2]    = -results.segments[i].conditions.frames.inertial.position_vector[j,2] 
                stencil_length         = S_gm_x*2 + 1
                stencil_width          = S_gm_y*2 + 1
                SPL_contour_gm[idx,int(S_locs[j,0]):int(S_locs[j,1]),int(S_locs[j,2]):int(S_locs[j,3])]  = results.segments[i].conditions.noise.total_SPL_dBA[j].reshape(stencil_length ,stencil_width )  
                if N_bm > 0:
                    SPL_contour_bm[idx,:]  = results.segments[i].conditions.noise.total_SPL_dBA[j,-N_bm:]  
    
    noise_data                        = Data()
    noise_data.SPL_dBA_ground_mic     = np.nan_to_num(SPL_contour_gm)
    noise_data.SPL_dBA_building_mic   = np.nan_to_num(SPL_contour_bm)
    noise_data.aircraft_position      = Aircraft_pos
    noise_data.SPL_dBA_ground_mic_loc = Mic_pos_gm 
    noise_data.N_gm_y                 = N_gm_y
    noise_data.N_gm_x                 = N_gm_x  
    noise_data.N_gm                   = N_gm_x* N_gm_y
    
    return noise_data 

 
# ------------------------------------------------------------------
# Plot Flight Profile Noise Contours 
# ------------------------------------------------------------------
def plot_noise_contour(res,PP,save_fig_name):    


    noise_data = post_process_noise_data(res)     

    SPL_contour_gm  = noise_data.SPL_dBA_ground_mic        
    Aircraft_pos    = noise_data.aircraft_position       
    X               = noise_data.SPL_dBA_ground_mic_loc[:,:,0]  
    Y               = noise_data.SPL_dBA_ground_mic_loc[:,:,1]  
    Z               = noise_data.SPL_dBA_ground_mic_loc[:,:,2]   
    
    threshold_dbA = 55 
    max_SPL_gm    = np.max(SPL_contour_gm,axis=0)
    threshold_idx = np.where(max_SPL_gm.flatten() > threshold_dbA)[0]
    print( 'Percentage over thresholds: ' + str( round(len(threshold_idx)*100/noise_data.N_gm, 2)))
    
    # ---------------------------------------------------------------------------
    # Full Noise Contour 
    # ---------------------------------------------------------------------------
    filename            = 'Noise_Contour' + save_fig_name 
    fig                 = plt.figure(filename)
    fig.set_size_inches(9,6)
    min_SPL             = 35
    max_SPL             = 80    
    levs                = np.linspace(min_SPL,max_SPL,10)
    axes                = fig.add_subplot(1,1,1) 
    CS                  = axes.contourf(X,Y,max_SPL_gm, levels  = levs, cmap=plt.cm.jet, extend='both')
    cbar = fig.colorbar(CS)
    cbar.ax.set_ylabel('SPL (dBA)', rotation =  90)
    axes.set_ylabel('y (m)',labelpad = 12)
    axes.set_xlabel('x (m)')  
    plt.tight_layout()
    plt.savefig(filename + '.png')     
 
    return 
   
# ----------------------------------------------------------------------
#  Load results
# ----------------------------------------------------------------------     
def load_results(filename):
    load_file =   filename + '.pkl'
    with open(load_file, 'rb') as file:
        results = pickle.load(file)
        
    return  results 

if __name__ == '__main__': 
    main()    
    plt.show()
    
    



