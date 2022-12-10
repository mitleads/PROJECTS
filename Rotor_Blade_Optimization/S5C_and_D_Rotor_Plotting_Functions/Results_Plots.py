import SUAVE 
from SUAVE.Core import Units , Data  

# Package Imports 
import matplotlib.cm as cm 
import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker 
import matplotlib.colors as colors  
from matplotlib.cm import ScalarMappable
from mpl_toolkits.mplot3d import Axes3D  
from SUAVE.Methods.Propulsion                                          import lift_rotor_design,  propeller_design 
from SUAVE.Analyses.Mission.Segments.Segment                           import Segment 
from SUAVE.Methods.Noise.Fidelity_One.Propeller.propeller_mid_fidelity import propeller_mid_fidelity
from SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics           import Aerodynamics 
from SUAVE.Components.Energy.Networks.Battery_Propeller                import Battery_Propeller  
from SUAVE.Components.Energy.Converters                                import Lift_Rotor , Prop_Rotor
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.generate_interpolated_airfoils import generate_interpolated_airfoils 

from SUAVE.Components.Energy.Networks.Battery_Propeller                import Battery_Propeller 
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry import import_airfoil_geometry
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_naca_4series    import compute_naca_4series
import os
import pickle
  


# ------------------------------------------------------------------ 
# Define plot parameters 
# ------------------------------------------------------------------  
def define_plot_parameters(): 

    plt.rcParams['axes.linewidth'] = 2.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 32,
                  'legend.fontsize': 22,
                  'xtick.labelsize': 28,
                  'ytick.labelsize': 28,
                  'axes.titlesize': 32}
    plt.rcParams.update(parameters)
    plot_parameters                  = Data()
    plot_parameters.line_width       = 2
    plot_parameters.line_styles      = ['-',':','--',':','--']
    plot_parameters.figure_width     = 10
    plot_parameters.figure_height    = 7
    plot_parameters.marker_size      = 10
    plot_parameters.legend_font_size = 20
    plot_parameters.alpha_val        = 0.25
    plot_parameters.root_color       = 'grey'
    plot_parameters.plot_grid        = True   

    plot_parameters.colors           = [['black','firebrick','darkblue'],
                                        ['dimgray','red','blue'], 
                                        ['darkgray','salmon','deepskyblue']]  

    plot_parameters.colors_1         = ['black','darkmagenta','mediumblue','darkgreen','darkgoldenrod','darkred']   
    plot_parameters.colors_2         = ['grey','orchid','darkcyan','green','orange','red']        
    plot_parameters.markers          = ['s','o','v','P','p','^','D','X','*']   
    
    return plot_parameters 


# ------------------------------------------------------------------ 
# Setup Axes 
# ------------------------------------------------------------------ 
def set_up_axes(PP,rotor_tag):
    
    # ------------------------------------------------------------------
    #   Twist Distribition
    # ------------------------------------------------------------------
    fig_1_name =  rotor_tag + "_Twist_Comparison"  
    fig_1 = plt.figure(fig_1_name)
    fig_1.set_size_inches(PP.figure_width,PP.figure_height)
    axis_1 = fig_1.add_subplot(1,1,1)
    axis_1.set_ylabel(r'$\beta$ ($\degree$)') 
    axis_1.set_xlabel('r')    
    axis_1.minorticks_on()   
    
    # ------------------------------------------------------------------
    #   Chord Distribution
    # ------------------------------------------------------------------ 
    fig_2_name =   rotor_tag + "_Chord_Comparison"  
    fig_2 = plt.figure(fig_2_name)     
    fig_2.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_2 = fig_2.add_subplot(1,1,1)  
    axis_2.set_ylabel('c (m)') 
    axis_2.set_xlabel('r')    
    axis_2.minorticks_on()    

    # ------------------------------------------------------------------
    #  Thickness Distribution
    # ------------------------------------------------------------------ 
    fig_3_name =  rotor_tag + "_Thickness_Comparison"  
    fig_3 = plt.figure(fig_3_name)     
    fig_3.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_3 = fig_3.add_subplot(1,1,1)  
    axis_3.set_ylabel('t (m)') 
    axis_3.set_xlabel('r')    
    axis_3.minorticks_on()   

    # ------------------------------------------------------------------
    # Thrust Comparison
    # ------------------------------------------------------------------      
    fig_4_name =  rotor_tag + "Thrust_Comparison" 
    fig_4 = plt.figure(fig_4_name)    
    fig_4.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_4= fig_4.add_subplot(1,1,1)    
    axis_4.set_ylabel(r'T (N)') 
    axis_4.set_xlabel('r')  

    # ------------------------------------------------------------------
    # Torque Comparison
    # ------------------------------------------------------------------   
    fig_5_name =   rotor_tag + "_Torque_Comparison"  
    fig_5 = plt.figure(fig_5_name)            
    fig_5.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_5 = fig_5.add_subplot(1,1,1)    
    axis_5.set_ylabel(r'Q (N-m)') 
    axis_5.set_xlabel('r')   

    # ------------------------------------------------------------------
    # Noise Requency Spectrum 
    # ------------------------------------------------------------------ 
    fig_6_name =  rotor_tag + "_Total_SPL_Comparison" 
    fig_6 = plt.figure(fig_6_name)            
    fig_6.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_6 = fig_6.add_subplot(1,1,1)    
    axis_6.set_xscale('log') 
    axis_6.set_ylabel(r'SPL$_{1/3}$ (dBA)')
    axis_6.set_xlabel('Frequency (Hz)') 
    axis_6.set_ylim([0,100])  

    # ------------------------------------------------------------------
    # Performance Pareto 1
    # ------------------------------------------------------------------       
    fig_7_name =  rotor_tag + "_Power_Noise_Pareto"  
    fig_7 = plt.figure(fig_7_name)     
    fig_7.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_7 = fig_7.add_subplot(1,1,1)  
    axis_7.set_xlabel('Power (kW)') 
    axis_7.set_ylabel('SPL (dBA)')    
    axis_7.minorticks_on()  
 

    # ------------------------------------------------------------------
    # Performance Pareto 2
    # ------------------------------------------------------------------    
    fig_8_name =  rotor_tag + "_Power_RPM_Pareto"  
    fig_8 = plt.figure(fig_8_name)     
    fig_8.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_8 = fig_8.add_subplot(1,1,1)  
    axis_8.set_xlabel('Power (kW)') 
    axis_8.set_ylabel('RPM')    
    axis_8.minorticks_on()  
    
    
    AXES    = [axis_1,axis_2,axis_3,axis_4,axis_5,axis_6,axis_7,axis_8]
    FIGURES = [fig_1,fig_2,fig_3,fig_4,fig_5,fig_6,fig_7,fig_8]
    return AXES , FIGURES 


 
# ------------------------------------------------------------------ 
# Plot lift rotor pareto fronteir 
# ------------------------------------------------------------------ 
def plot_rotor_blade_comparisons(rotor,folder_name,alpha_weights = None,beta_weights = None,gamma_weights = None,add_plot_legends = False , save_figures = True):     
    PP              = define_plot_parameters() 
     
    PP.colors_1     = cm.viridis(np.linspace(0,1,len(alpha_weights)))    
    
    if type(rotor) == Prop_Rotor: 
        AXES , FIGURES  = set_up_axes(PP,'PR') 
        
        # file names 
        fig_1_name = "PR_Twist_Compairson"  
        fig_2_name = "PR_Chord_Compairson" 
        fig_3_name = "PR_Thickness_Comparison"   
        fig_4_name = 'PR_Thrust_Comparison' 
        fig_5_name = 'PR_Torque_Comparison'    
        fig_6_name = 'PR_Total_SPL_Comparison'   
        fig_7_name = "PR_Power_Noise_Pareto"   
        fig_8_name = 'PR_Power_RPM_Pareto'  
        
        for i in range(len(alpha_weights)):  
            for j in range(len(beta_weights)):  
                alpha_opt_weight = str(format(alpha_weights[i],'.5f'))
                alpha_opt_weight = alpha_opt_weight.replace('.','_')    
                beta_opt_weight  = str(format(beta_weights[j],'.5f'))
                beta_opt_weight  = beta_opt_weight.replace('.','_')    
                rotor_file_name  = 'PR_Alpha_' + alpha_opt_weight + '_Beta_' + beta_opt_weight   
                rotor_label      = r'$\alpha$ = ' + str(alpha_weights[i])  + r' $\beta$ = ' + str(beta_weights[j])   
 
                try: 
                    rotor       = load_blade_geometry(folder_name,rotor_file_name)
                    plot_rotor_2d_geoemtry_and_performance(rotor,AXES,PP.colors_1[i],PP.line_styles[0],
                                                           PP.line_styles[1],PP.markers[0],PP.markers[1],rotor_label)
                except: 
                    pass   
             
        
    elif type(rotor) == Lift_Rotor:

        AXES , FIGURES  = set_up_axes(PP,'LR') 
        
        # file names 
        fig_1_name = "LR_Twist_Compairson" 
        fig_2_name = "LR_Chord_Compairson"  
        fig_3_name = "LR_Thickness_Comparison"   
        fig_4_name = 'LR_Thrust_Comparison' 
        fig_5_name = 'LR_Torque_Comparison'   
        fig_6_name = 'LR_Total_SPL_Comparison'   
        fig_7_name = "LR_Power_Noise_Pareto"  
        fig_8_name = 'LR_Power_RPM_Pareto' 
        
        try:   
            # Plot Rotor designed using Adkins and Liebeck  
            rotor_file_name  = 'LR_AL'  
            rotor_label      = 'Adkins & Liebeck'
            rotor            = load_blade_geometry(folder_name,rotor_file_name)       
            plot_rotor_2d_geoemtry_and_performance(rotor,AXES,'black',PP.line_styles[0],PP.line_styles[1],PP.markers[0],PP.markers[1],rotor_label)      
        except: 
            pass        
        
        for i in range(len(alpha_weights)):    
            # save rotor geomtry
            alpha_opt_weight = str(format(alpha_weights[i],'.5f'))
            alpha_opt_weight = alpha_opt_weight.replace('.','_')     
            rotor_file_name  =  'LR_Alpha_' + alpha_opt_weight   
            rotor_label      = r'$\alpha$ = ' + str(alpha_weights[i])  
            #try: 
            rotor       = load_blade_geometry(folder_name,rotor_file_name)
            plot_rotor_2d_geoemtry_and_performance(rotor,AXES,PP.colors_1[i],PP.line_styles[0],
                                                       PP.line_styles[1],PP.markers[0],PP.markers[1],rotor_label) 
            #except: 
               # pass   
            
    if add_plot_legends:        
        AXES[0].legend(loc='upper right')
        AXES[1].legend(loc='upper right')
        AXES[2].legend(loc='upper right')
        AXES[3].legend(loc='upper right')
        AXES[4].legend(loc='upper right')
        AXES[5].legend(loc='upper right')
        AXES[6].legend(loc='upper right')
        AXES[7].legend(loc='upper right')
        
    
    # axis limits 
    ymin0, ymax0 = AXES[0].get_ylim()
    ymin1, ymax1 = AXES[1].get_ylim()
    ymin2, ymax2 = AXES[2].get_ylim()
    ymin3, ymax3 = AXES[3].get_ylim()
    ymin4, ymax4 = AXES[4].get_ylim()
    ymin5, ymax5 = AXES[5].get_ylim() 
    ymin6, ymax6 = AXES[6].get_ylim()
    ymin7, ymax7 = AXES[7].get_ylim()
    
    xmin6, xmax6 = AXES[6].get_xlim()
    xmin7, xmax7 = AXES[7].get_xlim() 
    
    
    AXES[0].set_ylim([ymin0, ymax0])    
    AXES[1].set_ylim([ymin1, ymax1])    
    AXES[2].set_ylim([ymin2, ymax2])    
    AXES[3].set_ylim([ymin3, ymax3])  
    AXES[4].set_ylim([ymin4, ymax4])     
    AXES[5].set_ylim([ymin5, ymax5]) 
    AXES[6].set_ylim([ymin6, ymax6])     
    AXES[7].set_ylim([ymin7, ymax7]) 

    AXES[6].set_xlim([xmin6, xmax6])     
    AXES[7].set_xlim([xmin7, xmax7])    
    
    label_location = 0.5

    # axis labels 
    AXES[0].text(0.02,  (ymax0+ymin0)*label_location, 'Root', fontsize=25)    
    AXES[1].text(0.02,  (ymax1+ymin1)*label_location, 'Root', fontsize=25)  
    AXES[2].text(0.02,  (ymax2+ymin2)*label_location, 'Root', fontsize=25)   
    AXES[3].text(0.02,  (ymax3+ymin3)*label_location, 'Root', fontsize=25) 
    AXES[4].text(0.02 , (ymax4+ymin4)*label_location, 'Root', fontsize=25)     
                            
    # Assign Colormap and save plots 
    cmap     = plt.get_cmap("viridis")
    new_cmap = truncate_colormap(cmap, 0.0, 1.0)
    norm     = plt.Normalize(0,1) 
    sm       =  ScalarMappable(norm=norm, cmap=new_cmap)
    ax_ticks = np.linspace(0,1,11)
    sm.set_array([])  

    cmap_2      = plt.get_cmap("viridis")
    new_cmap_2  = truncate_colormap(cmap_2, 0.0, 1.0)
    norm_2      = plt.Normalize(0.45,0.7) 
    sm_2        = ScalarMappable(norm=norm_2, cmap=new_cmap_2 )
    ax_ticks_2  = np.linspace(0.45,0.7,6)
    sm_2.set_array([])     
            
    sfmt = ticker.ScalarFormatter(useMathText=True) 
    sfmt = ticker.FormatStrFormatter('%.1f')     
    sfmt2 = ticker.ScalarFormatter(useMathText=True) 
    sfmt2 = ticker.FormatStrFormatter('%.2f')     
    cbar_1 = FIGURES[0].colorbar(sm,   ax = AXES[0], ticks = list(ax_ticks),  format= sfmt)
    cbar_2 = FIGURES[1].colorbar(sm,   ax = AXES[1], ticks = list(ax_ticks),  format= sfmt)
    cbar_3 = FIGURES[2].colorbar(sm,   ax = AXES[2], ticks = list(ax_ticks),  format= sfmt)
    cbar_4 = FIGURES[3].colorbar(sm,   ax = AXES[3], ticks = list(ax_ticks),  format= sfmt)
    cbar_5 = FIGURES[4].colorbar(sm,   ax = AXES[4], ticks = list(ax_ticks),  format= sfmt) 
    cbar_6 = FIGURES[5].colorbar(sm,   ax = AXES[5], ticks = list(ax_ticks),  format= sfmt)
    cbar_7 = FIGURES[6].colorbar(sm,   ax = AXES[6], ticks = list(ax_ticks),  format= sfmt)
    cbar_8 = FIGURES[7].colorbar(sm_2, ax = AXES[7], ticks = list(ax_ticks_2),  format= sfmt2)
    
    
    cbar_1.set_label(r'$\alpha$')
    cbar_2.set_label(r'$\alpha$')
    cbar_3.set_label(r'$\alpha$')
    cbar_4.set_label(r'$\alpha$') 
    cbar_5.set_label(r'$\alpha$')  
    cbar_6.set_label(r'$\alpha$')
    cbar_7.set_label(r'$\alpha$') 
    cbar_8.set_label(r'Tip Mach')   
     
    FIGURES[0].tight_layout()
    FIGURES[1].tight_layout()
    FIGURES[2].tight_layout()
    FIGURES[3].tight_layout()
    FIGURES[4].tight_layout() 
    FIGURES[5].tight_layout()
    FIGURES[6].tight_layout(rect= (0.05,0,1,1))
    FIGURES[7].tight_layout(rect= (0.05,0,1,1))  
    
    if save_figures:
        FIGURES[0].savefig(fig_1_name  + '.pdf')               
        FIGURES[1].savefig(fig_2_name  + '.pdf')               
        FIGURES[2].savefig(fig_3_name  + '.pdf')               
        FIGURES[3].savefig(fig_4_name  + '.pdf')          
        FIGURES[4].savefig(fig_5_name  + '.pdf')                
        FIGURES[5].savefig(fig_6_name  + '.pdf')               
        FIGURES[6].savefig(fig_7_name  + '.pdf')            
        FIGURES[7].savefig(fig_8_name  + '.pdf')    
     
    return   

# ------------------------------------------------------------------ 
# Plot geoemtry and performance of single rotor 
# ------------------------------------------------------------------  
def plot_rotor_2d_geoemtry_and_performance(rotor,AXES,color,line_style,line_style_2, marker,marker_2,label = 'prop'):   
    PP       = define_plot_parameters()

    n_root_sections   = 1 # 8
    rotor_modified    = rotor # add_rotor_stem(rotor,number_of_root_sections = n_root_sections)
    c                 = rotor_modified.chord_distribution
    beta              = rotor_modified.twist_distribution/Units.degrees 
    r                 = rotor_modified.radius_distribution/rotor.tip_radius
    t                 = rotor_modified.max_thickness_distribution 
    
    if type(rotor_modified) == Prop_Rotor: 
        prop_rotor_flag    = True 
        T_hover            = rotor_modified.hover.design_performance.blade_thrust_distribution[0] 
        Q_hover            = rotor_modified.hover.design_performance.blade_torque_distribution[0]  
        SPL_dBA_1_3_hover  = rotor_modified.hover.design_noise.SPL_1_3_spectrum_dBA[0,0] 
        frequency          = rotor_modified.hover.design_noise.one_third_frequency_spectrum  
        RPM                = rotor_modified.hover.design_angular_velocity/Units.rpm  
        SPL_max            = rotor_modified.hover.design_SPL_dBA 

        T_cruise           = rotor_modified.cruise.design_performance.blade_thrust_distribution[0] 
        Q_cruise           = rotor_modified.cruise.design_performance.blade_torque_distribution[0]  
        SPL_dBA_1_3_cruise = rotor_modified.cruise.design_noise.SPL_1_3_spectrum_dBA[0,0]   
        PM_cruise          = rotor_modified.cruise.collective_pitch
        design_power       = rotor_modified.cruise.design_power 

    elif type(rotor) == Lift_Rotor: 
        prop_rotor_flag   = False 
        T_hover           = rotor_modified.hover.design_performance.blade_thrust_distribution[0] 
        Q_hover           = rotor_modified.hover.design_performance.blade_torque_distribution[0]  
        SPL_dBA_1_3_hover = rotor_modified.hover.design_noise.SPL_1_3_spectrum_dBA[0,0] 
        frequency         = rotor_modified.hover.design_noise.one_third_frequency_spectrum   
        RPM               = rotor_modified.hover.design_angular_velocity/Units.rpm 
        SPL_max           = rotor_modified.hover.design_SPL_dBA
        design_power      = rotor_modified.hover.design_power
     
    AXES[0].axvspan(0, r[n_root_sections-1], alpha=PP.alpha_val, color= PP.root_color)
    AXES[0].plot(r[:(n_root_sections)], beta[:(n_root_sections)],color = 'grey', linestyle =  line_style,linewidth = PP.line_width)  
    AXES[0].plot(r[(n_root_sections-1):], beta[(n_root_sections-1):],color = color , markersize = PP.marker_size ,marker = marker, linestyle = line_style,linewidth = PP.line_width, label = label )  
    AXES[0].set_ylabel(r'$\beta$ ($\degree$)') 
    AXES[0].set_xlabel('r')    
    AXES[0].set_xlim([0,max(r)])   
     
    AXES[1].axvspan(0, r[n_root_sections-1], alpha=PP.alpha_val, color= PP.root_color)
    AXES[1].plot(r[:(n_root_sections)], c[:(n_root_sections)] ,color = 'grey', linestyle =  line_style,linewidth = PP.line_width)  
    AXES[1].plot(r[(n_root_sections-1):], c[(n_root_sections-1):] ,color = color,marker = marker , markersize = PP.marker_size, linestyle = line_style,linewidth = PP.line_width, label = label )    
    AXES[1].set_ylabel('c (m)') 
    AXES[1].set_xlim([0,max(r)]) 
    AXES[1].set_xlabel('r')    
 
    AXES[2].axvspan(0, r[n_root_sections-1], alpha=PP.alpha_val, color= PP.root_color)
    AXES[2].plot(r[:(n_root_sections)], t[:(n_root_sections)] ,color = 'grey', linestyle = line_style,linewidth = PP.line_width)  
    AXES[2].plot(r[(n_root_sections-1):] , t[(n_root_sections-1):],color = color,marker = marker, markersize = PP.marker_size, linestyle = line_style,linewidth = PP.line_width, label = label )     
    AXES[2].set_ylabel('t (m)')  
    AXES[2].set_xlim([0,max(r)])   
    AXES[2].set_xlabel('r')    
         
     
    AXES[3].axvspan(0, r[n_root_sections-1], alpha=PP.alpha_val, color= PP.root_color)
    if prop_rotor_flag: 
        AXES[3].plot(r[(n_root_sections-1):] , T_hover ,color = color , markersize = PP.marker_size,marker = marker, linestyle =line_style, linewidth = PP.line_width , label = label + ' Hover'  )   
        AXES[3].plot(r[(n_root_sections-1):] , T_cruise,color = color , markersize = PP.marker_size,marker = marker_2, linestyle =line_style_2, linewidth = PP.line_width, label = label + ' Cruise'  )    
    else: 
        AXES[3].plot(r[(n_root_sections-1):] , T_hover ,color = color, markersize = PP.marker_size,marker = marker, linestyle = line_style, linewidth = PP.line_width , label = label  )    
         
    AXES[3].set_xlim([0,max(r)])   
    
     
    AXES[4].axvspan(0, r[n_root_sections-1], alpha=PP.alpha_val, color= PP.root_color) 
    if prop_rotor_flag: 
        AXES[4].plot(r[(n_root_sections-1):] , Q_hover ,color = color , markersize = PP.marker_size,marker = marker, linestyle =line_style, linewidth = PP.line_width , label = label + ' Hover')   
        AXES[4].plot(r[(n_root_sections-1):] , Q_cruise,color = color , markersize = PP.marker_size,marker = marker_2, linestyle =line_style_2, linewidth = PP.line_width, label = label + ' Cruise'  )    


    else: 
        AXES[4].plot(r[(n_root_sections-1):] , Q_hover ,color = color , markersize = PP.marker_size,marker = marker, linestyle = line_style, linewidth = PP.line_width , label = label  )            
    AXES[4].set_xlim([0,max(r)])   
 
    if prop_rotor_flag: 
        AXES[5].semilogx(frequency , SPL_dBA_1_3_hover ,color = color , markersize = PP.marker_size,marker = marker, linestyle = line_style, linewidth = PP.line_width, label = label   )    
    else: 
        AXES[5].semilogx(frequency , SPL_dBA_1_3_hover,color = color  , markersize = PP.marker_size,marker = marker, linestyle =line_style, linewidth = PP.line_width , label = label  )    
    
    AXES[6].scatter(design_power/1E3,SPL_max, color  = color, marker = 'o', s      = 150, label  = label)  
       
    AXES[7].scatter(design_power/1E3, RPM, color  = color,  marker = 'o',  s      = 150,  label  = label)  
    return   


# ------------------------------------------------------------------ 
# Truncate colormaps
# ------------------------------------------------------------------  
def truncate_colormap(cmap, minval=0.0, maxval=1.0, n=100):
    new_cmap = colors.LinearSegmentedColormap.from_list(
        'trunc({n},{a:.2f},{b:.2f})'.format(n=cmap.name, a=minval, b=maxval),
        cmap(np.linspace(minval, maxval, n)))
    return new_cmap

  
# ------------------------------------------------------------------ 
# Plot rotor 
# ------------------------------------------------------------------ 
def plot_3d_rotor_geometry(rotor,fig_name,save_figure, elevation_angle = 45,  aximuth_angle = 0, cpt=0,rotor_face_color='dodgerblue',
                        rotor_edge_color='darkblue',rotor_alpha=1.0):  
 
    n_root_sections   = 8
    rotor             = add_rotor_stem(rotor,number_of_root_sections = n_root_sections)
    
    fig_name = "3D_" + fig_name
    fig = plt.figure(fig_name) 
    fig.set_size_inches(12,7) 
    axes = plt.axes(projection='3d')  
    axes.view_init(elev= elevation_angle, azim= aximuth_angle)    
    RADIUS = 1.5 # Control this value.
    axes.set_xlim3d(-RADIUS / 2, RADIUS / 2)
    axes.set_zlim3d(-RADIUS / 2, RADIUS / 2)
    axes.set_ylim3d(-RADIUS / 2, RADIUS / 2)    
    axes.grid(False)  
    plt.axis('off')
        
    num_B     = rotor.number_of_blades
    n_points  = 21
    af_pts    = n_points-1
    dim       = len(rotor.radius_distribution) 

    for i in range(num_B):
        G = get_blade_coordinates(rotor,n_points,dim,i)
        # ------------------------------------------------------------------------
        # Plot Propeller Blade
        # ------------------------------------------------------------------------
        for sec in range(dim-1):
            for loc in range(af_pts):
                X = [G.XA1[cpt,sec,loc],
                     G.XB1[cpt,sec,loc],
                     G.XB2[cpt,sec,loc],
                     G.XA2[cpt,sec,loc]]
                Y = [G.YA1[cpt,sec,loc],
                     G.YB1[cpt,sec,loc],
                     G.YB2[cpt,sec,loc],
                     G.YA2[cpt,sec,loc]]
                Z = [G.ZA1[cpt,sec,loc],
                     G.ZB1[cpt,sec,loc],
                     G.ZB2[cpt,sec,loc],
                     G.ZA2[cpt,sec,loc]]
                rotor_verts = [list(zip(X, Y, Z))]
                rotor_collection = Poly3DCollection(rotor_verts)
                rotor_collection.set_facecolor(rotor_face_color)
                rotor_collection.set_edgecolor(rotor_edge_color)
                rotor_collection.set_alpha(rotor_alpha)
                axes.add_collection3d(rotor_collection)  
    
    if save_figure:
        fig.savefig(fig_name  + '.pdf') 
        
    return

# ------------------------------------------------------------------ 
# get blade geometry 
# ------------------------------------------------------------------ 
def get_blade_coordinates(rotor,n_points,dim,i,aircraftRefFrame=True): 
    # unpack
    num_B        = rotor.number_of_blades
    airfoils     = rotor.Airfoils 
    beta         = rotor.twist_distribution + rotor.inputs.pitch_command
    a_o          = rotor.start_angle
    b            = rotor.chord_distribution
    r            = rotor.radius_distribution
    MCA          = rotor.mid_chord_alignment
    t            = rotor.max_thickness_distribution
    a_loc        = rotor.airfoil_polar_stations
    origin       = rotor.origin
    
    if rotor.rotation==1:
        # negative chord and twist to give opposite rotation direction
        b = -b    
        beta = -beta
    
    theta  = np.linspace(0,2*np.pi,num_B+1)[:-1]
    flip_1 =  (np.pi/2)
    flip_2 =  (np.pi/2)

    MCA_2d             = np.repeat(np.atleast_2d(MCA).T,n_points,axis=1)
    b_2d               = np.repeat(np.atleast_2d(b).T  ,n_points,axis=1)
    t_2d               = np.repeat(np.atleast_2d(t).T  ,n_points,axis=1)
    r_2d               = np.repeat(np.atleast_2d(r).T  ,n_points,axis=1)
    airfoil_le_offset  = np.repeat(b[:,None], n_points, axis=1)/2  

    # get airfoil coordinate geometry
    if len(airfoils.keys())>0:
        xpts  = np.zeros((dim,n_points))
        zpts  = np.zeros((dim,n_points))
        max_t = np.zeros(dim)
        for af_idx,airfoil in enumerate(airfoils):
            geometry     = import_airfoil_geometry(airfoil.coordinate_file,n_points)
            locs         = np.where(np.array(a_loc) == af_idx)
            xpts[locs]   = geometry.x_coordinates  
            zpts[locs]   = geometry.y_coordinates  
            max_t[locs]  = geometry.thickness_to_chord 

    else: 
        airfoil_data = compute_naca_4series('2410',n_points)
        xpts         = np.repeat(np.atleast_2d(airfoil_data.x_coordinates) ,dim,axis=0)
        zpts         = np.repeat(np.atleast_2d(airfoil_data.y_coordinates) ,dim,axis=0)
        max_t        = np.repeat(airfoil_data.thickness_to_chord,dim,axis=0)
            
    # store points of airfoil in similar format as Vortex Points (i.e. in vertices)
    max_t2d = np.repeat(np.atleast_2d(max_t).T ,n_points,axis=1)

    xp      = (- MCA_2d + xpts*b_2d - airfoil_le_offset)     # x-coord of airfoil
    yp      = r_2d*np.ones_like(xp)                          # radial location
    zp      = zpts*(t_2d/max_t2d)                            # former airfoil y coord
    
    rotor_vel_to_body = rotor.prop_vel_to_body()
    cpts              = len(rotor_vel_to_body[:,0,0])
    
    matrix        = np.zeros((len(zp),n_points,3)) # radial location, airfoil pts (same y)
    matrix[:,:,0] = xp
    matrix[:,:,1] = yp
    matrix[:,:,2] = zp
    matrix        = np.repeat(matrix[None,:,:,:], cpts, axis=0)

    
    # ROTATION MATRICES FOR INNER SECTION
    # rotation about y axis to create twist and position blade upright
    trans_1        = np.zeros((dim,3,3))
    trans_1[:,0,0] = np.cos(flip_1 - beta)
    trans_1[:,0,2] = -np.sin(flip_1 - beta)
    trans_1[:,1,1] = 1
    trans_1[:,2,0] = np.sin(flip_1 - beta)
    trans_1[:,2,2] = np.cos(flip_1 - beta)
    trans_1        = np.repeat(trans_1[None,:,:,:], cpts, axis=0)

    # rotation about x axis to create azimuth locations
    trans_2 = np.array([[1 , 0 , 0],
                   [0 , np.cos(theta[i] + a_o + flip_2 ), -np.sin(theta[i] +a_o +  flip_2)],
                   [0,np.sin(theta[i] + a_o + flip_2), np.cos(theta[i] + a_o + flip_2)]])
    trans_2 = np.repeat(trans_2[None,:,:], dim, axis=0)
    trans_2 = np.repeat(trans_2[None,:,:,:], cpts, axis=0)

    # rotation about y to orient propeller/rotor to thrust angle (from propeller frame to aircraft frame)
    trans_3 =  rotor_vel_to_body
    trans_3 =  np.repeat(trans_3[:, None,:,: ],dim,axis=1) 
    
    trans     = np.matmul(trans_2,trans_1)
    rot_mat   = np.repeat(trans[:,:, None,:,:],n_points,axis=2)    

    # ---------------------------------------------------------------------------------------------
    # ROTATE POINTS
    if aircraftRefFrame:
        # rotate all points to the thrust angle with trans_3
        mat  =  np.matmul(np.matmul(rot_mat,matrix[...,None]).squeeze(axis=-1), trans_3)
    else:
        # use the rotor frame
        mat  =  np.matmul(rot_mat,matrix[...,None]).squeeze(axis=-1)
    # ---------------------------------------------------------------------------------------------
    # create empty data structure for storing geometry
    G = Data()
    
    # store node points
    G.X  = mat[:,:,:,0] + origin[0][0]
    G.Y  = mat[:,:,:,1] + origin[0][1]
    G.Z  = mat[:,:,:,2] + origin[0][2]
    
    # store points
    G.XA1  = mat[:,:-1,:-1,0] + origin[0][0]
    G.YA1  = mat[:,:-1,:-1,1] + origin[0][1]
    G.ZA1  = mat[:,:-1,:-1,2] + origin[0][2]
    G.XA2  = mat[:,:-1,1:,0]  + origin[0][0]
    G.YA2  = mat[:,:-1,1:,1]  + origin[0][1]
    G.ZA2  = mat[:,:-1,1:,2]  + origin[0][2]

    G.XB1  = mat[:,1:,:-1,0] + origin[0][0]
    G.YB1  = mat[:,1:,:-1,1] + origin[0][1]
    G.ZB1  = mat[:,1:,:-1,2] + origin[0][2]
    G.XB2  = mat[:,1:,1:,0]  + origin[0][0]
    G.YB2  = mat[:,1:,1:,1]  + origin[0][1]
    G.ZB2  = mat[:,1:,1:,2]  + origin[0][2]
    
    return G

# ------------------------------------------------------------------ 
# Add rotor stem 
# ------------------------------------------------------------------  
def add_rotor_stem(rotor,number_of_root_sections= 5):
    
    # define airfoil sections   
    a1            = "Circle_Section.txt"
    a2            = rotor.Airfoils[list(rotor.Airfoils.keys())[0]].coordinate_file    # first airfoil on rotor 
    new_files     = generate_interpolated_airfoils(a1, a2, number_of_root_sections,save_filename="Root_Airfoil")  
    
    for i in range(number_of_root_sections-1):
        # import geometry  
        airfoil                     = SUAVE.Components.Airfoils.Airfoil()
        airfoil.coordinate_file     = new_files[i]         
        airfoil.tag                 = 'Root_Section_' + str(i)
        airfoil.geometry            = import_airfoil_geometry(airfoil.coordinate_file )
        # append geometry
        rotor.Airfoils.append(airfoil) 
    
    # modify rotor 
    x      = np.linspace(0,4,number_of_root_sections)  
    func_1 = (np.tanh(x-2) + 2)/3
    func_2 = (np.tanh(x-2) + 1)/3 
    
    root_radius = np.linspace(0.1,rotor.radius_distribution[0],number_of_root_sections)[:-1]
    root_chord  = func_1[:-1]*rotor.chord_distribution[0]
    root_twist  = func_2[:-1]*rotor.twist_distribution[0]
    root_aloc   = list(np.arange(1,number_of_root_sections)) 
 
    # update rotor geoetry  
    rotor.airfoil_polar_stations     = root_aloc + rotor.airfoil_polar_stations
    rotor.chord_distribution         = np.hstack(( root_chord, rotor.chord_distribution   )) 
    rotor.twist_distribution         = np.hstack((root_twist , rotor.twist_distribution  ))   
    rotor.radius_distribution        = np.hstack((root_radius , rotor.radius_distribution       ))  
    rotor.mid_chord_alignment        = np.hstack((np.ones(number_of_root_sections - 1)*rotor.mid_chord_alignment[0] , rotor.mid_chord_alignment       ))  
    
    t_max_root  = np.zeros((number_of_root_sections - 1))    
    t_c_root    = np.zeros((number_of_root_sections - 1))    
    if len(rotor.Airfoils.keys())>0:
        for j,airfoil in enumerate(rotor.Airfoils): 
            a_geo              = airfoil.geometry
            locs               = np.where(np.array(root_aloc) == j )
            t_max_root[locs]   = a_geo.thickness_to_chord*rotor.chord_distribution[locs]
            t_c_root[locs]     = a_geo.thickness_to_chord      
     
    rotor.max_thickness_distribution = np.hstack(( t_max_root, rotor.max_thickness_distribution)) 
    rotor.thickness_to_chord         = np.hstack((t_c_root , rotor.thickness_to_chord        ))  
    
    return rotor   


# ------------------------------------------------------------------ 
# Load data  
# ------------------------------------------------------------------     
def load_blade_geometry(folder_name,filename):  
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator     
    load_file = rel_path + separator  + '../' + folder_name + '/Rotor_Designs/' + filename + '.pkl'
    with open(load_file, 'rb') as file:
        rotor = pickle.load(file) 
    return rotor
