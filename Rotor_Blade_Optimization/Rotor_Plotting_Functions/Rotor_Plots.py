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
# Prop-Rotor Design Comparisons
# ------------------------------------------------------------------ 
def plot_prop_rotor_design_comparisons(alpha_weights,beta_weights,use_pyoptsparse_flag,save_figures): 
    PP         = define_plot_parameters()
    design_thrust_hover  = (2300*9.81/(8))    
    design_thrust_cruise = 1410/8

    alpha_0 = str(alpha_weights[0])
    alpha_0 = alpha_0.replace('.','_')    
    beta_0  = str(beta_weights[0])
    beta_0  = beta_0.replace('.','_')     
    alpha_1 = str(alpha_weights[1])
    alpha_1 = alpha_1.replace('.','_')    
    beta_1  = str(beta_weights[1])
    beta_1  = beta_1.replace('.','_')       

    rotor_comparison = '_A' + alpha_0 + '_B' +  beta_0 + '_A' + alpha_1 + '_B'+  beta_1
    
    fig_1_name = "Thrust_TW_Rotor_Comparison" + rotor_comparison
    fig_2_name = "Torque_TW_Rotor_Comparison" + rotor_comparison
    fig_3_name = "Blade_Re_TW_Rotor_Comparison" + rotor_comparison 
    fig_4_name = "Blade_AoA_TW_Rotor_Comparison" + rotor_comparison
    fig_5_name = 'SPL_TW_Rotor_Comparison' + rotor_comparison 
    fig_6_name = "Twist_TW_Rotor_Comparison" + rotor_comparison 
    fig_7_name = "Chord_TW_Rotor_Comparison"  + rotor_comparison
    fig_8_name = "Thickness_TW_Rotor_Comparison" + rotor_comparison 
    fig_9_name = "MCA_TW_Rotor_Comparison" + rotor_comparison 
      
    fig_1 = plt.figure(fig_1_name)
    fig_1.set_size_inches(PP.figure_width, PP.figure_height)     
    axis_1 = fig_1.add_subplot(1,1,1)   
    axis_1.set_xlabel('r')
    axis_1.set_ylabel('T (N)')    
    
    fig_2 = plt.figure(fig_2_name) 
    fig_2.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_2 = fig_2.add_subplot(1,1,1)    
    axis_2.set_xlabel('r')
    axis_2.set_ylabel('Q (N-m)') 
    
    fig_3 = plt.figure(fig_3_name)      
    fig_3.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_3 = fig_3.add_subplot(1,1,1) 
    axis_3.set_xlabel('r')
    axis_3.set_ylabel(r'Re.')  

    fig_4 = plt.figure(fig_4_name)   
    fig_4.set_size_inches(PP.figure_width, PP.figure_height)    
    axis_4 = fig_4.add_subplot(1,1,1)   
    axis_4.set_ylabel(r'AoA$_{eff}$ ($\degree$)') 
    axis_4.set_ylim([-5,35])
    axis_4.set_xlabel('r')
        
    fig_5 = plt.figure(fig_5_name)    
    fig_5.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_5 = fig_5.add_subplot(1,1,1)  
    axis_5.set_ylabel(r'SPL$_{1/3}$ (dBA)')
    axis_5.set_xlabel('Frequency (Hz)') 
        
    fig_6 = plt.figure(fig_6_name)
    fig_6.set_size_inches(PP.figure_width, PP.figure_height)     
    axis_6 = fig_6.add_subplot(1,1,1)  
    axis_6.set_ylabel(r'$\beta$ ($\degree$)') 
    axis_6.set_xlabel('r')      
    
    fig_7 = plt.figure(fig_7_name)
    fig_7.set_size_inches(PP.figure_width, PP.figure_height)   
    axis_7 = fig_7.add_subplot(1,1,1)
    axis_7.set_ylabel('c (m)') 
    axis_7.set_xlabel('r')    
    
    fig_8 = plt.figure(fig_8_name)
    fig_8.set_size_inches(PP.figure_width, PP.figure_height)       
    axis_8 = fig_8.add_subplot(1,1,1)
    axis_8.set_ylabel('t (m)')  
    axis_8.set_xlabel('r')      
    
    
    fig_9 = plt.figure(fig_9_name)
    fig_9.set_size_inches(PP.figure_width, PP.figure_height)       
    axis_9 = fig_9.add_subplot(1,1,1)  
    axis_9.set_ylabel('M.C.A (m)')  
    axis_9.set_xlabel('r')   
    
    
    ospath               = os.path.abspath(__file__)
    separator            = os.path.sep
    rel_path             = os.path.dirname(ospath) + separator  
    angles               = np.array([135]) 
    folder               = 'Rotor_Designs' 
    
    if use_pyoptsparse_flag:
        optimizer = 'SNOPT'
    else: 
        optimizer = 'SLSQP'
    
    for i in range(len(alpha_weights)): 
                
        # save rotor geomtry
        alpha_opt_weight = str(alpha_weights[i])
        alpha_opt_weight = alpha_opt_weight.replace('.','_')    
        beta_opt_weight  = str(beta_weights[i])
        beta_opt_weight  = beta_opt_weight.replace('.','_')    
        file_name        =  rel_path +  folder + separator + 'Rotor_TH_' + str(int(design_thrust_hover)) + '_TC_' + str(int(design_thrust_cruise)) +\
                         '_Alpha_' + alpha_opt_weight + '_Beta_' + beta_opt_weight + '_Opt_' + optimizer  
        rotor            = load_blade_geometry(file_name)
        rotor_name       =  'Rotor_TH_' + str(int(design_thrust_hover)) + '_TC_' + str(int(design_thrust_cruise)) +\
                           '_Alpha_' + alpha_opt_weight + '_Beta_' + beta_opt_weight + '_Opt_' + optimizer 
 
        c    = rotor.chord_distribution
        beta = rotor.twist_distribution
        MCA  = rotor.mid_chord_alignment
        r    = rotor.radius_distribution/rotor.tip_radius
        t    = rotor.max_thickness_distribution
       
        T_hover           = rotor.design_performance_hover.blade_thrust_distribution[0] 
        Q_hover           = rotor.design_performance_hover.blade_torque_distribution[0] 
        Re_hover          = rotor.design_performance_hover.blade_reynolds_number_distribution[0]  
        AoA_hover         = rotor.design_performance_hover.blade_effective_angle_of_attack[0]/Units.degrees 
        SPL_dBA_1_3_hover = rotor.design_acoustics_hover.SPL_1_3_spectrum_dBA 
        frequency         = rotor.design_acoustics_hover.one_third_frequency_spectrum 
        RPM_hover         = rotor.angular_velocity_hover/Units.rpm
        PM_hover          = rotor.inputs.pitch_command_hover

        T_cruise           = rotor.design_performance_cruise.blade_thrust_distribution[0] 
        Q_cruise           = rotor.design_performance_cruise.blade_torque_distribution[0] 
        Re_cruise          = rotor.design_performance_cruise.blade_reynolds_number_distribution[0]  
        AoA_cruise         = rotor.design_performance_cruise.blade_effective_angle_of_attack[0]/Units.degrees 
        SPL_dBA_1_3_cruise = rotor.design_acoustics_cruise.SPL_1_3_spectrum_dBA  
        RPM_cruise         = rotor.angular_velocity_cruise/Units.rpm
        PM_cruise          = rotor.inputs.pitch_command_cruise 
        
        
        TM_cruise = rotor.angular_velocity_cruise*rotor.tip_radius/343    
        TM_hover = rotor.angular_velocity_hover*rotor.tip_radius/343    
        # ----------------------------------------------------------------------------
        # 2D - Plots     
        # ----------------------------------------------------------------------------  
        rotor_label         = r'$\alpha$ = ' +  str(alpha_weights[i]) + r', $\beta$ = ' + str(beta_weights[i])
        hover_text          = r'Hover        : RPM = ' + str(int(RPM_hover)) + ' TM = ' + str(TM_hover) +  r', Pitch Command = ' + str(int(PM_hover/Units.degrees)) + r' degrees '    
        cruise_text         = r'Cruise       : RPM = ' + str(int(RPM_cruise)) + ' TM = ' + str(TM_cruise)  + r', Pitch Command = ' + str(int(PM_cruise/Units.degrees)) + r' degrees ' 
        SPL_text            =  'SPL          : ' + str(rotor.design_SPL_dBA_hover)
        cruise_Pow_text     =  'Cruise Power : ' + str(rotor.design_performance_cruise.power[0][0])
        hover_Pow_text      =  'Hover Power  : ' + str(rotor.design_performance_hover.power[0][0])
        print('Alpha = ' +  str(alpha_weights[i]) + ', Beta = ' + str(beta_weights[i]))
        print(hover_text)   
        print(hover_Pow_text)
        print(cruise_text) 
        print(cruise_Pow_text)
        print(SPL_text)
        
        hover_label  = r'Hover: ' + rotor_label
        cruise_label = r'Cruise: ' + rotor_label
  
        axis_1.plot(r , T_hover ,color = PP.colors[0][i]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width, label =  hover_label )   
        axis_1.plot(r , T_cruise ,color = PP.colors[1][i] , markersize = PP.marker_size,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label =  cruise_label  )            
  
        axis_2.plot(r , Q_hover , color = PP.colors[0][i]  , markersize = PP.marker_size ,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_label)  
        axis_2.plot(r , Q_cruise ,color =  PP.colors[1][i] , markersize = PP.marker_size ,marker =  PP.markers[0], linestyle = PP.line_styles[0], linewidth = PP.line_width,label =  cruise_label  )            
  
        axis_3.plot(r , Re_hover ,color = PP.colors[0][i] , markersize = PP.marker_size ,marker =PP.markers[2],linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_label)    
        axis_3.plot(r , Re_cruise ,color =  PP.colors[1][i]  , markersize = PP.marker_size,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label =   cruise_label  )       
 
        axis_4.plot(r , AoA_hover ,color = PP.colors[0][i] , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_label )  
        axis_4.plot(r , AoA_cruise ,color = PP.colors[1][i]  , markersize = PP.marker_size,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label =  cruise_label   )         
       
        mic = 0
        hover_noise_label  = r'Hover: $\theta_{mic}$ = ' + str(int(angles[mic])) + r'$\degree$, ' + rotor_label
        #cruise_noise_label = r'Cruise: $\theta_{mic}$ = ' + str(int(angles[mic])) + r' $\degree$' + rotor_label
        axis_5.semilogx(frequency , SPL_dBA_1_3_hover[0,mic] ,color = PP.colors[0][i]   , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_noise_label )  
        #axis_5.semilogx(frequency , SPL_dBA_1_3_cruise[0,mic],color = PP.colors[1][i] , markersize = PP.marker_size  ,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label = cruise_noise_label )        
  
  
        axis_6.plot(r, beta/Units.degrees,color = PP.colors[0][i] , markersize = PP.marker_size ,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width, label = rotor_label)  
  
        axis_7.plot(r, c ,color = PP.colors[0][i]  ,marker = PP.markers[2] , markersize = PP.marker_size, linestyle = PP.line_styles[2],linewidth = PP.line_width, label = rotor_label)    
    
        axis_8.plot(r , t,color = PP.colors[0][i]  ,marker = PP.markers[2] , markersize = PP.marker_size, linestyle = PP.line_styles[2],linewidth = PP.line_width, label = rotor_label)     
        
        axis_9.plot(r, MCA,color = PP.colors[0][i]  ,marker = PP.markers[2] , markersize = PP.marker_size, linestyle = PP.line_styles[2],linewidth = PP.line_width, label = rotor_label)  
    
        
        # plot propeller geometry
        plot_3d_rotor_geometry(rotor,rotor_name,save_figures)
         
    
    axis_1.set_ylim([-40,250]) 
    axis_2.set_ylim([-20,40]) 
    axis_3.set_ylim([35E3,35E5])   
    axis_4.set_ylim([-20,10])  
    axis_5.set_ylim([0,120]) 
    axis_6.set_ylim([-10,60]) 
    axis_7.set_ylim([0.0,0.40]) 
    axis_8.set_ylim([0.0,0.05]) 
    axis_9.set_ylim([-0.06,0.03]) 
    
    axis_1.legend(loc='upper left')    
    axis_2.legend(loc='upper left')    
    axis_3.legend(loc='upper left')  
    axis_4.legend(loc='lower right')     
    axis_5.legend(loc='upper right') 
    axis_6.legend(loc='upper right')     
    axis_7.legend(loc='upper right') 
    axis_8.legend(loc='upper right')     
    axis_9.legend(loc='upper right')   
    
    fig_1.tight_layout()
    fig_2.tight_layout()
    fig_3.tight_layout(rect= (0.05,0,1,1))
    fig_4.tight_layout()
    fig_5.tight_layout()
    fig_6.tight_layout()
    fig_7.tight_layout(rect= (0.05,0,1,1))
    fig_8.tight_layout(rect= (0.05,0,1,1))
    fig_9.tight_layout(rect= (0.05,0,1,1))   
      
    if save_figures:
        fig_1.savefig(fig_1_name  + '.pdf')               
        fig_2.savefig(fig_2_name  + '.pdf')               
        fig_3.savefig(fig_3_name  + '.pdf')               
        fig_4.savefig(fig_4_name  + '.pdf')              
        fig_5.savefig(fig_5_name  + '.pdf')       
        fig_6.savefig(fig_6_name  + '.pdf')        
        fig_7.savefig(fig_7_name  + '.pdf')               
        fig_8.savefig(fig_8_name  + '.pdf')               
        fig_9.savefig(fig_9_name  + '.pdf')   
    
    return   

# ------------------------------------------------------------------ 
# Plot Results and Pareto Fronteir
# ------------------------------------------------------------------ 
def prop_rotor_designs_and_pareto_fronteir(weight_1,weight_2,sweep_name,use_pyoptsparse_flag,save_figures):    
    PP         = define_plot_parameters()
    PP.colors            = cm.viridis(np.linspace(0,1,len(weight_2)))    
    design_thrust_hover  = 21356/(6-1) 
    design_thrust_cruise = 1400 
    folder               = 'Rotor_Designs'
    if use_pyoptsparse_flag:
        optimizer = 'SNOPT'
    else: 
        optimizer = 'SLSQP'
    ospath               = os.path.abspath(__file__)
    separator            = os.path.sep
    rel_path             = os.path.dirname(ospath) + separator   

    AXES , FIGURES = set_up_axes(PP,design_thrust_hover)
    axis_1  = AXES[0] 
    axis_2  = AXES[1] 
    axis_3  = AXES[2] 
    axis_4  = AXES[3] 
    axis_5  = AXES[4] 
    axis_6  = AXES[5] 
    axis_7  = AXES[6] 
    axis_8  = AXES[7] 
    axis_9  = AXES[8] 
    axis_10  = AXES[9] 
    fig_1   = FIGURES[0] 
    fig_2   = FIGURES[1] 
    fig_3   = FIGURES[2] 
    fig_4   = FIGURES[3] 
    fig_5   = FIGURES[4] 
    fig_6   = FIGURES[5] 
    fig_7   = FIGURES[6] 
    fig_8   = FIGURES[7] 
    fig_9   = FIGURES[8]
    fig_10  = FIGURES[9]
     
    for idx in range(len(weight_1) + 1):           
        rotor_flag = True
        if idx == 0: 
            rotor_file_name  = rel_path +  folder + separator + 'Rotor_T_' + str(int(design_thrust_hover)) + '_AL'   
            rotor_tag        = 'T:' + str(int(design_thrust_hover)) + 'A.& L.'
            rotor_name       =  'Adkins & Liebeck'            
            rotor            = load_blade_geometry(rotor_file_name)      
            
        else:
            # save rotor geomtry
            alpha_opt_weight = str(weight_1[idx-1])
            alpha_opt_weight = alpha_opt_weight.replace('.','_')    
            beta_opt_weight  = str(weight_2[idx-1])
            beta_opt_weight  = beta_opt_weight.replace('.','_')    
            rotor_file_name = rel_path +  folder + separator + 'Rotor_TH_' + str(int(design_thrust_hover)) + '_TC_' + str(int(design_thrust_cruise)) +\
                         '_Alpha_' + alpha_opt_weight + '_Beta_' + beta_opt_weight + '_Opt_' + optimizer  
            try: 
                rotor       = load_blade_geometry(rotor_file_name)
                rotor_flag  = True 
                rotor_tag   = 'T:' + str(int(design_thrust_hover)) + r', $\alpha$' + str(weight_1[idx-1]) + r', $\beta$' + str(weight_2[idx-1])
                rotor_name  = r'$\alpha$ = ' + str(weight_1[idx-1]) + r' $\beta$ = ' + str(weight_2[idx-1])  
            except: 
                rotor_flag  = False  
        
        if rotor_flag:
            rotor_aero_data_hover    = rotor.design_performance_hover
            rotor_noise_data_hover   = rotor.design_acoustics_hover 
            Total_SPL_1_3            = rotor_noise_data_hover.SPL_1_3_spectrum_dBA
            Harmonic_1_3             = rotor_noise_data_hover.SPL_harmonic_1_3_spectrum_dBA
            Broadband_1_3            = rotor_noise_data_hover.SPL_broadband_1_3_spectrum_dBA 
            One_Third_Spectrum       = rotor_noise_data_hover.one_third_frequency_spectrum 
            if idx == 0: 
                axis_7.plot(One_Third_Spectrum , Total_SPL_1_3[0,0] , color = 'black' , linestyle = PP.line_styles[2], marker = PP.markers[idx] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_8.plot(One_Third_Spectrum , Harmonic_1_3[0,0]  , color = 'black' , linestyle = PP.line_styles[2], marker = PP.markers[idx] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_9.plot(One_Third_Spectrum , Broadband_1_3[0,0] , color = 'black' , linestyle = PP.line_styles[2], marker = PP.markers[idx] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)  
                propeller_geoemtry_comparison_plots(rotor,rotor_aero_data_hover,AXES,'black',PP,idx, rotor_name)  
            
            else: 
                axis_7.plot(One_Third_Spectrum , Total_SPL_1_3[0,0] , color = PP.colors[idx-1] , linestyle = PP.line_styles[2], marker = PP.markers[(idx-1)%9] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_8.plot(One_Third_Spectrum , Harmonic_1_3[0,0]  , color = PP.colors[idx-1] , linestyle = PP.line_styles[2], marker = PP.markers[(idx-1)%9] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_9.plot(One_Third_Spectrum , Broadband_1_3[0,0] , color = PP.colors[idx-1] , linestyle = PP.line_styles[2], marker = PP.markers[(idx-1)%9] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)  
                propeller_geoemtry_comparison_plots(rotor,rotor_aero_data_hover,AXES,PP.colors[idx-1],PP,idx-1, rotor_name)  
                
    
    cmap     = plt.get_cmap("viridis")
    norm     = plt.Normalize(0,1) 
    sm       =  ScalarMappable(norm=norm, cmap=cmap)
    ax_ticks = np.linspace(0,1,11)
    sm.set_array([]) 
            
    sfmt = ticker.ScalarFormatter(useMathText=True) 
    sfmt = ticker.FormatStrFormatter('%.1f')      
    cbar_1  = fig_1.colorbar(sm, ax = axis_1, ticks = list(ax_ticks),  format= sfmt)
    cbar_2  = fig_2.colorbar(sm, ax = axis_2, ticks = list(ax_ticks),  format= sfmt)
    cbar_3  = fig_3.colorbar(sm, ax = axis_3, ticks = list(ax_ticks),  format= sfmt)
    cbar_4  = fig_4.colorbar(sm, ax = axis_4, ticks = list(ax_ticks),  format= sfmt)
    cbar_5  = fig_5.colorbar(sm, ax = axis_5, ticks = list(ax_ticks),  format= sfmt)
    cbar_6  = fig_6.colorbar(sm, ax = axis_6, ticks = list(ax_ticks),  format= sfmt)
    cbar_7  = fig_7.colorbar(sm, ax = axis_7, ticks = list(ax_ticks),  format= sfmt)
    cbar_8  = fig_8.colorbar(sm, ax = axis_8, ticks = list(ax_ticks),  format= sfmt)
    cbar_9  = fig_9.colorbar(sm, ax = axis_9, ticks = list(ax_ticks),  format= sfmt)
    cbar_10 = fig_10.colorbar(sm, ax = axis_10, ticks = list(ax_ticks),  format= sfmt)
    
    
    cbar_1.set_label(r'$\alpha$')
    cbar_2.set_label(r'$\alpha$')
    cbar_3.set_label(r'$\alpha$')
    cbar_4.set_label(r'$\alpha$') 
    cbar_5.set_label(r'$\alpha$')
    cbar_6.set_label(r'$\alpha$')
    cbar_7.set_label(r'$\alpha$') 
    cbar_8.set_label(r'$\alpha$')
    cbar_9.set_label(r'$\alpha$') 
    cbar_10.set_label(r'$\alpha$')
    
    
    fig_1_name = "Rotor_Twist_Compairson_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    fig_2_name = "Rotor_Chord_Compairson_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    fig_3_name = "Rotor_Thickness_Comparison_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    fig_4_name = "Rotor_Hover_Power_Noise_Pareto_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    fig_5_name = "Rotor_Blade_Re_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    fig_6_name = "Rotor_Blade_AoA_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name    
    fig_7_name = 'Rotor_Total_SPL_Comparison' + sweep_name
    fig_8_name = 'Rotor_Harmonic_Noise_Comparison' + sweep_name
    fig_9_name = 'Rotor_Broadband_Noise_Comparison' + sweep_name 
    fig_10_name = "Rotor_Cruise_Power_Noise_Pareto_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    
    fig_1.tight_layout()
    fig_2.tight_layout()
    fig_3.tight_layout()
    fig_4.tight_layout()
    fig_5.tight_layout()
    fig_6.tight_layout()
    fig_7.tight_layout()
    fig_8.tight_layout()
    fig_9.tight_layout()  
    fig_10.tight_layout()   
    
    if save_figures:
        fig_1.savefig(fig_1_name  + '.pdf')               
        fig_2.savefig(fig_2_name  + '.pdf')               
        fig_3.savefig(fig_3_name  + '.pdf')               
        fig_4.savefig(fig_4_name  + '.pdf')              
        fig_5.savefig(fig_5_name  + '.pdf')       
        fig_6.savefig(fig_6_name  + '.pdf')        
        fig_7.savefig(fig_7_name  + '.pdf')               
        fig_8.savefig(fig_8_name  + '.pdf')               
        fig_9.savefig(fig_9_name  + '.pdf')                
        fig_10.savefig(fig_10_name  + '.pdf')    
    
    
    return  

 # ------------------------------------------------------------------ 
# Plot lift rotor pareto fronteir 
# ------------------------------------------------------------------ 
def plot_lift_rotor_pareto_fronteir(design_thrust,alpha_weights,use_pyoptsparse_flag,save_figures):     
    PP         = define_plot_parameters()
    PP.colors  = cm.hot(np.linspace(0,1,len(alpha_weights)))  
    folder     = '../Lift_Rotor_Design/Rotor_Designs'   
    if use_pyoptsparse_flag:
        optimizer = 'SNOPT'
    else: 
        optimizer = 'SLSQP'
    ospath               = os.path.abspath(__file__)
    separator            = os.path.sep
    rel_path             = os.path.dirname(ospath) + separator  

    AXES , FIGURES = set_up_axes(PP,design_thrust)
    axis_1  = AXES[0] 
    axis_2  = AXES[1] 
    axis_3  = AXES[2] 
    axis_4  = AXES[3] 
    axis_5  = AXES[4] 
    axis_6  = AXES[5] 
    axis_7  = AXES[6] 
    axis_8  = AXES[7] 
    axis_9  = AXES[8] 
    axis_10 = AXES[9] 
    axis_11 = AXES[10]
    axis_12 = AXES[11]  
    fig_1   = FIGURES[0] 
    fig_2   = FIGURES[1] 
    fig_3   = FIGURES[2] 
    fig_4   = FIGURES[3] 
    fig_5   = FIGURES[4] 
    fig_6   = FIGURES[5] 
    fig_7   = FIGURES[6] 
    fig_8   = FIGURES[7] 
    fig_9   = FIGURES[8]
    fig_10  = FIGURES[9] 
    fig_11  = FIGURES[10]
    fig_12  = FIGURES[11]
     
     
    for idx in range(len(alpha_weights) + 1):   
        rotor_flag = True
        if idx == 0: 
            rotor_file_name  =  rel_path +  folder + separator + 'Rotor_T_' + str(int(design_thrust)) + '_AL' 
            rotor_tag        = 'T:' + str(int(design_thrust)) + 'A.& L.'
            rotor_name       = 'Adkins & Liebeck'
            rotor            = load_blade_geometry(rotor_file_name)            
        else:
            # save rotor geomtry
            alpha_opt_weight = str(alpha_weights[idx-1])
            alpha_opt_weight = alpha_opt_weight.replace('.','_')     
            rotor_file_name  =  rel_path +  folder + separator +  'Rotor_T_' + str(int(rotor.design_thrust))  + '_Alpha_' + alpha_opt_weight + '_Opt_' + optimizer
            rotor_tag   = 'T:' + str(int(design_thrust)) + r', $\alpha$' + str(alpha_weights[idx-1])
            rotor_name  = r'$\alpha$ = ' + str(alpha_weights[idx-1]) 
            try: 
                rotor       = load_blade_geometry(rotor_file_name)
                rotor_flag  = True 
            except: 
                rotor_flag  = False  
        
        if rotor_flag:
            rotor_aero_data    = rotor.design_performance
            rotor_noise_data   = rotor.design_acoustics 
            Total_SPL_1_3      = rotor_noise_data.SPL_1_3_spectrum_dBA
            Harmonic_1_3       = rotor_noise_data.SPL_harmonic_1_3_spectrum_dBA
            Broadband_1_3      = rotor_noise_data.SPL_broadband_1_3_spectrum_dBA 
            One_Third_Spectrum = rotor_noise_data.one_third_frequency_spectrum 
                
            if idx == 0:
                pass
            else: 
                axis_7.plot(One_Third_Spectrum , Total_SPL_1_3[0,0] , color = PP.colors[idx-1] , linestyle = PP.line_styles[2], linewidth = PP.line_width,  label = rotor_tag)      
                axis_8.plot(One_Third_Spectrum , Harmonic_1_3[0,0]  , color = PP.colors[idx-1] , linestyle = PP.line_styles[2] , linewidth = PP.line_width,  label = rotor_tag)      
                axis_9.plot(One_Third_Spectrum , Broadband_1_3[0,0] , color = PP.colors[idx-1] , linestyle = PP.line_styles[2] , linewidth = PP.line_width,  label = rotor_tag)  
                
                propeller_geoemtry_comparison_plots(rotor,rotor_aero_data,AXES,PP.colors[idx-1], rotor_name)  
                
    
    cmap      = plt.get_cmap("hot")
    new_cmap = truncate_colormap(cmap, 0.0, 0.8)
    norm     = plt.Normalize(0,1) 
    sm       =  ScalarMappable(norm=norm, cmap=new_cmap)
    ax_ticks = np.linspace(0,1,11)
    sm.set_array([])  

    cmap_2      = plt.get_cmap("hot")
    new_cmap_2 = truncate_colormap(cmap_2, 0.0, 0.8)
    norm_2      = plt.Normalize(0.45,0.7) 
    sm_2        = ScalarMappable(norm=norm_2, cmap=new_cmap_2 )
    ax_ticks_2  = np.linspace(0.45,0.7,6)
    sm_2.set_array([])     
            
    sfmt = ticker.ScalarFormatter(useMathText=True) 
    sfmt = ticker.FormatStrFormatter('%.1f')     
    sfmt2 = ticker.ScalarFormatter(useMathText=True) 
    sfmt2 = ticker.FormatStrFormatter('%.2f')     
    cbar_1 = fig_1.colorbar(sm, ax = axis_1, ticks = list(ax_ticks),  format= sfmt)
    cbar_2 = fig_2.colorbar(sm, ax = axis_2, ticks = list(ax_ticks),  format= sfmt)
    cbar_3 = fig_3.colorbar(sm, ax = axis_3, ticks = list(ax_ticks),  format= sfmt)
    cbar_4 = fig_4.colorbar(sm, ax = axis_4, ticks = list(ax_ticks),  format= sfmt)
    cbar_5 = fig_5.colorbar(sm, ax = axis_5, ticks = list(ax_ticks),  format= sfmt)
    cbar_6 = fig_6.colorbar(sm, ax = axis_6, ticks = list(ax_ticks),  format= sfmt)
    cbar_7 = fig_7.colorbar(sm, ax = axis_7, ticks = list(ax_ticks),  format= sfmt)
    cbar_8 = fig_8.colorbar(sm, ax = axis_8, ticks = list(ax_ticks),  format= sfmt)
    cbar_9 = fig_9.colorbar(sm, ax = axis_9, ticks = list(ax_ticks),  format= sfmt)
    cbar_10 = fig_10.colorbar(sm, ax = axis_10, ticks = list(ax_ticks),  format= sfmt)
    cbar_11 = fig_11.colorbar(sm, ax = axis_11, ticks = list(ax_ticks),  format= sfmt)
    cbar_12 = fig_12.colorbar(sm_2, ax = axis_12, ticks = list(ax_ticks_2),  format= sfmt2)
    
    
    cbar_1.set_label(r'$\alpha$')
    cbar_2.set_label(r'$\alpha$')
    cbar_3.set_label(r'$\alpha$')
    cbar_4.set_label(r'$\alpha$') 
    cbar_5.set_label(r'$\alpha$')
    cbar_6.set_label(r'$\alpha$')
    cbar_7.set_label(r'$\alpha$') 
    cbar_8.set_label(r'$\alpha$')
    cbar_9.set_label(r'$\alpha$') 
    cbar_10.set_label(r'$\alpha$')
    cbar_11.set_label(r'$\alpha$') 
    cbar_12.set_label(r'Tip Mach')   
    
    fig_1_name = "Rotor_Twist_Compairson_" + str(int(design_thrust))  + '_N' 
    fig_2_name = "Rotor_Chord_Compairson_" + str(int(design_thrust))  + '_N' 
    fig_3_name = "Rotor_Thickness_Comparison_" + str(int(design_thrust))  + '_N' 
    fig_4_name = "Rotor_Power_Noise_Pareto_" + str(int(design_thrust))  + '_N'
    fig_5_name = "Rotor_Blade_Re_" + str(int(design_thrust))  + '_N' 
    fig_6_name = "Rotor_Blade_AoA_" + str(int(design_thrust))  + '_N'         
    fig_7_name = 'Rotor_Total_SPL_Comparison'
    fig_8_name = 'Rotor_Harmonic_Noise_Comparison'
    fig_9_name = 'Rotor_Broadband_Noise_Comparison'  
    fig_10_name = 'Rotor_Thrust_Comparison'+ str(int(design_thrust))  + '_N'
    fig_11_name = 'Rotor_Torque_Comparison' + str(int(design_thrust))  + '_N'
    fig_12_name = 'Rotor_RPM_Comparison' + str(int(design_thrust))  + '_N'
    
    fig_1.tight_layout()
    fig_2.tight_layout()
    fig_3.tight_layout()
    fig_4.tight_layout()
    fig_5.tight_layout()
    fig_6.tight_layout()
    fig_7.tight_layout()
    fig_8.tight_layout()
    fig_9.tight_layout() 
    fig_10.tight_layout()
    fig_11.tight_layout()  
    fig_12.tight_layout()    
    
    if save_figures:
        fig_1.savefig(fig_1_name  + '.pdf')               
        fig_2.savefig(fig_2_name  + '.pdf')               
        fig_3.savefig(fig_3_name  + '.pdf')               
        fig_4.savefig(fig_4_name  + '.pdf')              
        fig_5.savefig(fig_5_name  + '.pdf')       
        fig_6.savefig(fig_6_name  + '.pdf')        
        fig_7.savefig(fig_7_name  + '.pdf')               
        fig_8.savefig(fig_8_name  + '.pdf')               
        fig_9.savefig(fig_9_name  + '.pdf')               
        fig_10.savefig(fig_10_name  + '.pdf')               
        fig_11.savefig(fig_11_name  + '.pdf')            
        fig_12.savefig(fig_12_name  + '.pdf')    
    
    
    return   

# ------------------------------------------------------------------ 
# Plot rotor 
# ------------------------------------------------------------------ 
def plot_3d_rotor_geometry(rotor,fig_name,save_figure, elevation_angle = 45,  aximuth_angle = 0, cpt=0,rotor_face_color='dodgerblue',
                        rotor_edge_color='darkblue',rotor_alpha=1.0): 
    
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
    airfoils     = rotor.airfoils 
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
# Plot geoemtry and performance of single rotor 
# ------------------------------------------------------------------  
def plot_geoemtry_and_performance(rotor,rotor_name,save_figures):
    PP                = define_plot_parameters() 
    
    n_root_sections   = 8
    rotor_modified    = add_rotor_stem(rotor,number_of_root_sections = n_root_sections)
    c                 = rotor_modified.chord_distribution
    beta              = rotor_modified.twist_distribution
    MCA               = rotor_modified.mid_chord_alignment
    r                 = rotor_modified.radius_distribution/rotor.tip_radius
    t                 = rotor_modified.max_thickness_distribution 
    
    if type(rotor_modified) == Prop_Rotor: 
        prop_rotor_flag    = True 
        T_hover            = rotor_modified.design_performance_hover.blade_thrust_distribution[0] 
        Q_hover            = rotor_modified.design_performance_hover.blade_torque_distribution[0] 
        Re_hover           = rotor_modified.design_performance_hover.blade_reynolds_number[0]  
        AoA_hover          = rotor_modified.design_performance_hover.blade_effective_angle_of_attack[0]/Units.degrees 
        SPL_dBA_1_3_hover  = rotor_modified.design_acoustics_hover.SPL_1_3_spectrum_dBA[0,0] 
        frequency          = rotor_modified.design_acoustics_hover.one_third_frequency_spectrum
        PM_hover           = rotor_modified.inputs.pitch_command_hover

        T_cruise           = rotor_modified.design_performance_cruise.blade_thrust_distribution[0] 
        Q_cruise           = rotor_modified.design_performance_cruise.blade_torque_distribution[0] 
        Re_cruise          = rotor_modified.design_performance_cruise.blade_reynolds_number[0]  
        AoA_cruise         = rotor_modified.design_performance_cruise.blade_effective_angle_of_attack[0]/Units.degrees 
        SPL_dBA_1_3_cruise = rotor_modified.design_acoustics_cruise.SPL_1_3_spectrum_dBA[0,0]   
        PM_cruise          = rotor_modified.inputs.pitch_command_cruise

    elif type(rotor) == Lift_Rotor: 
        prop_rotor_flag    = False 
        T_hover           = rotor_modified.design_performance.blade_thrust_distribution[0] 
        Q_hover           = rotor_modified.design_performance.blade_torque_distribution[0] 
        Re_hover          = rotor_modified.design_performance.blade_reynolds_number[0]  
        AoA_hover         = rotor_modified.design_performance.blade_effective_angle_of_attack[0]/Units.degrees
        SPL_dBA_1_3_hover = rotor_modified.design_acoustics.SPL_1_3_spectrum_dBA[0,0] 
        frequency         = rotor_modified.design_acoustics.one_third_frequency_spectrum  
        PM_hover          = rotor_modified.inputs.pitch_command 
        
    # ----------------------------------------------------------------------------
    # 2D - Plots     
    # ----------------------------------------------------------------------------   
    
    fig_1 = plt.figure('Blade_Thurst_Distribution')
    fig_1.set_size_inches(PP.figure_width, PP.figure_height)     
    axis_1 = fig_1.add_subplot(1,1,1)    
    axis_1.axvspan(0, r[n_root_sections-1], alpha=PP.alpha_val, color= PP.root_color)
    if prop_rotor_flag: 
        axis_1.plot(r[(n_root_sections-1):] , T_hover ,color = PP.colors[0][0]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width  )   
        axis_1.plot(r[(n_root_sections-1):] , T_cruise,color = PP.colors[0][1]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width  )    
        min_y  = np.minimum(min(T_hover),min(T_cruise))
        max_y  = np.maximum(max(T_hover),max(T_cruise))
        axis_1.set_ylim([min_y*0.9 ,max_y*1.1 ]) 
        axis_1.text(0.05,max_y, 'Root', fontsize=25) 
    else: 
        axis_1.plot(r[(n_root_sections-1):] , T_hover ,color = PP.colors[0][0]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width  )    
        min_y  = min(T_hover)
        max_y  = max(T_hover)        
        axis_1.set_ylim([min_y*0.9 ,max_y*1.1 ]) 
        axis_1.text(0.05,max(T_hover), 'Root', fontsize=25) 
    axis_1.set_xlabel('r')
    axis_1.set_ylabel('T (N)')     
    axis_1.set_xlim([0,max(r)])   
    
    
    
    fig_2 = plt.figure('Blade_Torque_Distribution') 
    fig_2.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_2 = fig_2.add_subplot(1,1,1)  
    axis_2.axvspan(0, r[n_root_sections-1], alpha=PP.alpha_val, color= PP.root_color) 
    if prop_rotor_flag: 
        axis_2.plot(r[(n_root_sections-1):] , Q_hover ,color = PP.colors[0][0]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width  )   
        axis_2.plot(r[(n_root_sections-1):] , Q_cruise,color = PP.colors[0][1]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width  )    
        min_y  = np.minimum(min(Q_hover),min(Q_cruise))
        max_y  = np.maximum(max(Q_hover),max(Q_cruise))
        axis_2.set_ylim([min_y*0.9 ,max_y*1.1 ]) 
        axis_2.text(0.05,max_y, 'Root', fontsize=25) 
    else: 
        axis_2.plot(r[(n_root_sections-1):] , Q_hover ,color = PP.colors[0][0]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width  )    
        min_y  = min(Q_hover)
        max_y  = max(Q_hover)        
        axis_2.set_ylim([min_y*0.9 ,max_y*1.1 ]) 
        axis_2.text(0.05,max_y, 'Root', fontsize=25)   
    axis_2.set_xlabel('r')
    axis_2.set_ylabel('Q (N-m)')   
    axis_2.set_xlim([0,max(r)])     



    fig_3 = plt.figure('Blade_Re_Distribution')      
    fig_3.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_3 = fig_3.add_subplot(1,1,1)
    axis_3.axvspan(0, r[n_root_sections-1], alpha=PP.alpha_val, color= PP.root_color)
    if prop_rotor_flag: 
        axis_3.plot(r[(n_root_sections-1):] , Re_hover ,color = PP.colors[0][0]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width  )   
        axis_3.plot(r[(n_root_sections-1):] , Re_cruise,color = PP.colors[0][1]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width  )    
        min_y  = np.minimum(min(Re_hover),min(Re_cruise))
        max_y  = np.maximum(max(Re_hover),max(Re_cruise))
        axis_3.set_ylim([min_y*0.9 ,max_y*1.1 ]) 
        axis_3.text(0.05,max_y, 'Root', fontsize=25) 
    else: 
        axis_3.plot(r[(n_root_sections-1):] , Re_hover ,color = PP.colors[0][0]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width  )    
        min_y  = min(Re_hover)
        max_y  = max(Re_hover)        
        axis_3.set_ylim([min_y*0.9 ,max_y*1.1 ]) 
        axis_3.text(0.05,max_y, 'Root', fontsize=25)       
    axis_3.set_xlabel('r')
    axis_3.set_ylabel(r'Re.')     
    axis_3.set_xlim([0,max(r)])   



    fig_4 = plt.figure('Blade_AoA_Distribution')   
    fig_4.set_size_inches(PP.figure_width, PP.figure_height)    
    axis_4 = fig_4.add_subplot(1,1,1)   
    axis_4.axvspan(0, r[n_root_sections-1], alpha=PP.alpha_val, color= PP.root_color)
    if prop_rotor_flag: 
        axis_4.plot(r[(n_root_sections-1):] , AoA_hover ,color = PP.colors[0][0]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width  )   
        axis_4.plot(r[(n_root_sections-1):] , AoA_cruise,color = PP.colors[0][1]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width  )    
        min_y  = np.minimum(min(AoA_hover),min(AoA_cruise))
        max_y  = np.maximum(max(AoA_hover),max(AoA_cruise))
        axis_4.set_ylim([min_y*0.9 ,max_y*1.1 ]) 
        axis_4.text(0.05,max_y*0.9, 'Root', fontsize=25) 
    else: 
        axis_3.plot(r[(n_root_sections-1):] , AoA_hover ,color = PP.colors[0][0]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width  )    
        min_y  = min(AoA_hover)
        max_y  = max(AoA_hover)        
        axis_4.set_ylim([min_y*0.9 ,max_y*1.1 ]) 
        axis_4.text(0.05,max_y, 'Root', fontsize=25) 
    axis_4.set_xlabel('r')
    axis_4.set_ylabel(r'AoA$_{eff}$ (deg.)')   
    axis_4.set_xlim([0,max(r)])    



    fig_5 = plt.figure('Blade_SPL_dBA')    
    fig_5.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_5 = fig_5.add_subplot(1,1,1)  

    if prop_rotor_flag: 
        axis_5.semilogx(frequency , SPL_dBA_1_3_hover ,color = PP.colors[0][0]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width  )   
        axis_5.semilogx(frequency , SPL_dBA_1_3_cruise,color = PP.colors[0][1]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width  )    
        min_y  = 0
        max_y  = np.maximum(max(SPL_dBA_1_3_hover),max(SPL_dBA_1_3_cruise))
        axis_5.set_ylim([min_y*0.9 ,max_y*1.1 ]) 
    else: 
        axis_5.semilogx(frequency , SPL_dBA_1_3_hover,color = PP.colors[0][0]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width  )    
        min_y  = 0
        max_y  = max(SPL_dBA_1_3_hover)
        axis_5.set_ylim([min_y*0.9 ,max_y*1.1 ])  
    axis_5.set_ylabel(r'SPL$_{1/3}$ (dBA)')
    axis_5.set_xlabel('Frequency (Hz)') 
    axis_5.legend(loc='upper right')
    
        
    fig_6 = plt.figure('Twist_Distribution')
    fig_6.set_size_inches(PP.figure_width, PP.figure_height) 
    fig_6.set_size_inches(PP.figure_width, PP.figure_height)    
    axis_6 = fig_6.add_subplot(1,1,1)
    axis_6.axvspan(0, r[n_root_sections-1], alpha=PP.alpha_val, color= PP.root_color)
    axis_6.plot(r[:(n_root_sections)], beta[:(n_root_sections)]/Units.degrees,color = 'grey', linestyle =  PP.line_styles[2],linewidth = PP.line_width)  
    axis_6.plot(r[(n_root_sections-1):], beta[(n_root_sections-1):]/Units.degrees,color = PP.colors[0][0]  , markersize = PP.marker_size ,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width)  
    axis_6.set_ylabel(r'$\beta$ ($\degree$)') 
    axis_6.set_xlabel('r')    
    axis_6.text(0.05, max(beta/Units.degrees)*0.9, 'Root', fontsize=25) 
    axis_6.set_xlim([0,max(r)])   
    
    fig_7 = plt.figure('Chord_Distribution')
    fig_7.set_size_inches(PP.figure_width, PP.figure_height)   
    axis_7 = fig_7.add_subplot(1,1,1)
    axis_7.axvspan(0, r[n_root_sections-1], alpha=PP.alpha_val, color= PP.root_color)
    axis_7.plot(r[:(n_root_sections)], c[:(n_root_sections)] ,color = 'grey', linestyle =  PP.line_styles[2],linewidth = PP.line_width)  
    axis_7.plot(r[(n_root_sections-1):], c[(n_root_sections-1):] ,color = PP.colors[0][0]  ,marker = PP.markers[2] , markersize = PP.marker_size, linestyle = PP.line_styles[2],linewidth = PP.line_width)    
    axis_7.set_ylabel('c (m)') 
    axis_7.set_xlim([0,max(r)]) 
    axis_7.text(0.05, max(c)*0.9, 'Root', fontsize=25)   
    axis_7.set_xlabel('r')    

    fig_8 = plt.figure('Thickness_Distribution')
    fig_8.set_size_inches(PP.figure_width, PP.figure_height)       
    axis_8 = fig_8.add_subplot(1,1,1)
    axis_8.axvspan(0, r[n_root_sections-1], alpha=PP.alpha_val, color= PP.root_color)
    axis_8.plot(r[:(n_root_sections)], t[:(n_root_sections)] ,color = 'grey', linestyle =  PP.line_styles[2],linewidth = PP.line_width)  
    axis_8.plot(r[(n_root_sections-1):] , t[(n_root_sections-1):],color = PP.colors[0][0]  ,marker = PP.markers[2] , markersize = PP.marker_size, linestyle = PP.line_styles[2],linewidth = PP.line_width)     
    axis_8.set_ylabel('t (m)')  
    axis_8.set_xlim([0,max(r)])   
    axis_8.text(0.05, max(t)*0.1, 'Root', fontsize=25) 
    axis_8.set_xlabel('r')    

    fig_9 = plt.figure('MCA_Distribution')
    fig_9.set_size_inches(PP.figure_width, PP.figure_height)       
    axis_9 = fig_9.add_subplot(1,1,1)
    axis_9.axvspan(0, r[n_root_sections-1], alpha=PP.alpha_val, color= PP.root_color)
    axis_9.plot(r[:(n_root_sections)], MCA[:(n_root_sections)] ,color = 'grey', linestyle = PP.line_styles[2],linewidth = PP.line_width)  
    axis_9.plot(r[(n_root_sections-1):], MCA[(n_root_sections-1):],color = PP.colors[0][0]  ,marker = PP.markers[2] , markersize = PP.marker_size, linestyle = PP.line_styles[2],linewidth = PP.line_width)  
    axis_9.set_ylabel('M.C.A (m)')  
    axis_9.text(0.05, min(MCA), 'Root', fontsize=25) 
    axis_9.set_xlim([0,max(r)])       
    axis_9.set_xlabel('r')    

    fig_1_name = "Thrust_" + rotor_name
    fig_2_name = "Torque_"+ rotor_name 
    fig_3_name = "Blade_Re_" + rotor_name
    fig_4_name = "Blade_AoA_"+ rotor_name 
    fig_5_name = 'SPL_'  + rotor_name 
    fig_6_name = "Twist_" + rotor_name
    fig_7_name = "Chord_" + rotor_name
    fig_8_name = "Thickness_" + rotor_name
    fig_9_name = "MCA_" + rotor_name 
    
    fig_1.tight_layout()
    fig_2.tight_layout()
    fig_3.tight_layout()
    fig_4.tight_layout()
    fig_5.tight_layout()
    fig_6.tight_layout()
    fig_7.tight_layout(rect= (0.05,0,1,1))
    fig_8.tight_layout(rect= (0.05,0,1,1))
    fig_9.tight_layout(rect= (0.05,0,1,1))   
     
    if save_figures:
        fig_1.savefig(fig_1_name  + '.pdf')               
        fig_2.savefig(fig_2_name  + '.pdf')               
        fig_3.savefig(fig_3_name  + '.pdf')               
        fig_4.savefig(fig_4_name  + '.pdf')              
        fig_5.savefig(fig_5_name  + '.pdf')       
        fig_6.savefig(fig_6_name  + '.pdf')        
        fig_7.savefig(fig_7_name  + '.pdf')               
        fig_8.savefig(fig_8_name  + '.pdf')               
        fig_9.savefig(fig_9_name  + '.pdf')    
     
    # plot propeller geometry
    plot_3d_rotor_geometry(rotor,rotor_name,save_figures)     
    
    return   
 

# ------------------------------------------------------------------ 
# Plot Propeller Comparison Results
# ------------------------------------------------------------------ 
def propeller_geoemtry_comparison_plots(rotor,outputs,AXES,color,label_name = 'prop'):   
    PP       = define_plot_parameters()
    axis_1   = AXES[0] 
    axis_2   = AXES[1] 
    axis_3   = AXES[2] 
    axis_4   = AXES[3] 
    axis_5   = AXES[4] 
    axis_6   = AXES[5]  
    axis_10  = AXES[9] 
    axis_11  = AXES[10] 
    axis_12  = AXES[11]            
    axis_1.plot(rotor.radius_distribution/rotor.tip_radius, rotor.twist_distribution/Units.degrees,
                color      = color, 
                linestyle  = PP.line_styles[2],
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_2.plot(rotor.radius_distribution/rotor.tip_radius, rotor.chord_distribution,
                color      = color, 
                linestyle  = PP.line_styles[2],
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_3.plot(rotor.radius_distribution/rotor.tip_radius  , rotor.max_thickness_distribution,
                color      = color, 
                linestyle  = PP.line_styles[2],
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_4.scatter(rotor.design_power/1E3, rotor.design_SPL_dBA,
                   color  = color,
                   marker = 'o',
                   s      = 150,
                   label  = label_name )     
    
    axis_5.plot(rotor.radius_distribution/rotor.tip_radius, outputs.blade_reynolds_number[0],
                color      = color, 
                linestyle  = PP.line_styles[2],
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_6.plot(rotor.radius_distribution/rotor.tip_radius, outputs.blade_effective_angle_of_attack[0]/Units.degrees,
                color      = color, 
                linestyle  = PP.line_styles[2],
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
     
    axis_10.plot(rotor.radius_distribution/rotor.tip_radius, outputs.blade_thrust_distribution[0],
                color      = color, 
                linestyle  = PP.line_styles[2],
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_11.plot(rotor.radius_distribution/rotor.tip_radius, outputs.blade_torque_distribution[0],
                color      = color, 
                linestyle  = PP.line_styles[2],
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name)     
    axis_12.scatter(rotor.design_power/1E3, rotor.design_SPL_dBA,
                   color  = color,
                   marker = 'o',
                   s      = 150,
                   label  = label_name )   

    return

# ------------------------------------------------------------------ 
# Setup Axes 
# ------------------------------------------------------------------ 
def set_up_axes(PP,design_thrust):
    
    # ------------------------------------------------------------------
    #   Twist Distribition
    # ------------------------------------------------------------------
    fig_1_name = "Rotor_Twist_Comparson_" + str(int(design_thrust))  + '_N'
    fig_1 = plt.figure(fig_1_name)
    fig_1.set_size_inches(PP.figure_width,PP.figure_height)
    axis_1 = fig_1.add_subplot(1,1,1)
    axis_1.set_ylabel(r'$\beta$ ($\degree$)') 
    axis_1.set_xlabel('r')    
    axis_1.minorticks_on()   
    
    # ------------------------------------------------------------------
    #   Chord Distribution
    # ------------------------------------------------------------------ 
    fig_2_name = "Rotor_Chord_Comparson_" + str(int(design_thrust))  + '_N'
    fig_2 = plt.figure(fig_2_name)     
    fig_2.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_2 = fig_2.add_subplot(1,1,1)  
    axis_2.set_ylabel('c (m)') 
    axis_2.set_xlabel('r')    
    axis_2.minorticks_on()    

    # ------------------------------------------------------------------
    #  Thickness Distribution
    # ------------------------------------------------------------------ 
    fig_3_name = "Rotor_Thickness_Comparson_" + str(int(design_thrust))  + '_N'
    fig_3 = plt.figure(fig_3_name)     
    fig_3.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_3 = fig_3.add_subplot(1,1,1)  
    axis_3.set_ylabel('t (m)') 
    axis_3.set_xlabel('r')    
    axis_3.minorticks_on()  

    # ------------------------------------------------------------------
    #  Thickness Distribution
    # ------------------------------------------------------------------ 
    fig_4_name = "Rotor_Power_Noise_Pareto_" + str(int(design_thrust))  + '_N'
    fig_4 = plt.figure(fig_4_name)     
    fig_4.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_4 = fig_4.add_subplot(1,1,1)  
    axis_4.set_xlabel('Power (kW)') 
    axis_4.set_ylabel('SPL (dBA)')    
    axis_4.minorticks_on()  
    
    # ------------------------------------------------------------------
    #  Spanwise Re Distribution
    # ------------------------------------------------------------------ 
    fig_5_name = "Rotor_Spanwise_Re_" + str(int(design_thrust))  + '_N'
    fig_5 = plt.figure(fig_5_name)     
    fig_5.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_5 = fig_5.add_subplot(1,1,1)  
    axis_5.set_ylabel(r'Re.') 
    axis_5.set_xlabel('r')    
    axis_5.minorticks_on()   
     

    # ------------------------------------------------------------------
    # Spanwise AoA
    # ------------------------------------------------------------------ 
    fig_6_name = "Rotor_Spanwise_AoA_" + str(int(design_thrust))  + '_N'
    fig_6 = plt.figure(fig_6_name)     
    fig_6.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_6 = fig_6.add_subplot(1,1,1)  
    axis_6.set_ylabel(r'AoA$_{eff}$ ($\degree$)') 
    axis_6.set_xlabel('r')      
    axis_6.minorticks_on()     
      

    # ------------------------------------------------------------------
    # Total SPL Spectrum Comparison
    # ------------------------------------------------------------------      
    fig_7 = plt.figure('Rotor_Total_SPL_Comparison')    
    fig_7.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_7 = fig_7.add_subplot(1,1,1)    
    axis_7.set_xscale('log') 
    axis_7.set_ylabel(r'SPL$_{1/3}$ (dBA)')
    axis_7.set_xlabel('Frequency (Hz)') 
    axis_7.set_ylim([0,100]) 

    # ------------------------------------------------------------------
    # Harmonic Noise Spectrum Comparison
    # ------------------------------------------------------------------  
    fig_8 = plt.figure('Rotor_Harmonic_Noise_Comparison') 
    fig_8.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_8 = fig_8.add_subplot(1,1,1)      
    axis_8.set_xscale('log')
    axis_8.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axis_8.set_xlabel('Frequency (Hz)')  
    axis_8.set_ylim([0,100]) 
    

    # ------------------------------------------------------------------
    # Broadband Noise Spectrum Comparison
    # ------------------------------------------------------------------      
    fig_9 = plt.figure('Rotor_Broadband_Noise_Comparison')    
    fig_9.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_9 = fig_9.add_subplot(1,1,1)    
    axis_9.set_xscale('log')
    axis_9.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axis_9.set_xlabel('Frequency (Hz)') 
    axis_9.set_ylim([0,100])
    

    # ------------------------------------------------------------------
    # Thrust Comparison
    # ------------------------------------------------------------------      
    fig_10 = plt.figure('Rotor_Thrust_Comparison')    
    fig_10.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_10 = fig_10.add_subplot(1,1,1)    
    axis_10.set_ylabel(r'T (N)') 
    axis_10.set_xlabel('r')     
     

    # ------------------------------------------------------------------
    # Torque Comparison
    # ------------------------------------------------------------------          
    fig_11 = plt.figure('Rotor_Torque_Comparison')    
    fig_11.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_11 = fig_11.add_subplot(1,1,1)    
    axis_11.set_ylabel(r'Q (N-m)') 
    axis_11.set_xlabel('r')          

    # ------------------------------------------------------------------
    #  Thickness Distribution
    # ------------------------------------------------------------------ 
    fig_12_name = "Rotor_Power_RPM_Pareto_" + str(int(design_thrust))  + '_N'
    fig_12 = plt.figure(fig_12_name)     
    fig_12.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_12 = fig_12.add_subplot(1,1,1)  
    axis_12.set_xlabel('Power (kW)') 
    axis_12.set_ylabel('SPL (dBA)')    
    axis_12.minorticks_on()  
    
    
    AXES    = [axis_1,axis_2,axis_3,axis_4,axis_5,axis_6,axis_7,axis_8,axis_9,axis_10,axis_11,axis_12]
    FIGURES = [fig_1,fig_2,fig_3,fig_4,fig_5,fig_6,fig_7,fig_8,fig_9,fig_10,fig_11,fig_12]
    return AXES , FIGURES

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
    plot_parameters.line_styles      = ['--',':','-',':','--']
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
    plot_parameters.markers          = ['o','v','s','P','p','^','D','X','*']   
    
    return plot_parameters 

 
# ------------------------------------------------------------------ 
# Add rotor stem 
# ------------------------------------------------------------------  
def add_rotor_stem(rotor,number_of_root_sections= 5):
    
    # define airfoil sections   
    a1            = "Circle_Section.txt"
    a2            = rotor.airfoils[list(rotor.airfoils.keys())[0]].coordinate_file    # first airfoil on rotor 
    new_files     = generate_interpolated_airfoils(a1, a2, number_of_root_sections,save_filename="Root_Airfoil")  
    
    for i in range(number_of_root_sections-1):
        # import geometry  
        airfoil                     = SUAVE.Components.Airfoils.Airfoil()
        airfoil.coordinate_file     = new_files[i]         
        airfoil.tag                 = 'Root_Section_' + str(i)
        airfoil.geometry            = import_airfoil_geometry(airfoil.coordinate_file )
        # append geometry
        rotor.airfoils.append(airfoil) 
    
    # modify rotor 
    x      = np.linspace(0,4,number_of_root_sections)  
    func_1 = (np.tanh(x-2) + 2)/3 
    func_2 = (np.tanh(x-2) + 1)/3 
    
    root_radius = np.linspace(0.1,rotor.radius_distribution[0],number_of_root_sections)[:-1]
    root_chord  = func_1[:-1]*rotor.chord_distribution[0]
    root_twist  = func_2[:-1]*rotor.twist_distribution[0]
    root_aloc   = list(np.arange(1,number_of_root_sections)) 
 
    # update rotor geoetry  
    rotor.airfoil_polar_stations          = root_aloc + rotor.airfoil_polar_stations
    rotor.chord_distribution         = np.hstack(( root_chord, rotor.chord_distribution   )) 
    rotor.twist_distribution         = np.hstack((root_twist , rotor.twist_distribution  ))   
    rotor.radius_distribution        = np.hstack((root_radius , rotor.radius_distribution       ))  
    rotor.mid_chord_alignment        = np.hstack((np.ones(number_of_root_sections - 1)*rotor.mid_chord_alignment[0] , rotor.mid_chord_alignment       ))  
    
    t_max_root  = np.zeros((number_of_root_sections - 1))    
    t_c_root    = np.zeros((number_of_root_sections - 1))    
    if len(rotor.airfoils.keys())>0:
        for j,airfoil in enumerate(rotor.airfoils): 
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
def load_blade_geometry(filename):  
    load_file = filename + '.pkl' 
    with open(load_file, 'rb') as file:
        rotor = pickle.load(file) 
    return rotor
# ------------------------------------------------------------------ 
# Truncate colormaps
# ------------------------------------------------------------------  
def truncate_colormap(cmap, minval=0.0, maxval=1.0, n=100):
    new_cmap = colors.LinearSegmentedColormap.from_list(
        'trunc({n},{a:.2f},{b:.2f})'.format(n=cmap.name, a=minval, b=maxval),
        cmap(np.linspace(minval, maxval, n)))
    return new_cmap
