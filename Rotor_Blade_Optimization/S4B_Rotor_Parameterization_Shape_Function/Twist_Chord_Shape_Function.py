import SUAVE 
from SUAVE.Core import Units , Data  

# Package Imports 
import matplotlib.cm as cm 
import numpy as np 
import matplotlib.pyplot as plt 

# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main():
    

    # Add to rotor definition  

    plt.rcParams['axes.linewidth'] = 2.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 32,
                  'legend.fontsize': 22,
                  'xtick.labelsize': 28,
                  'ytick.labelsize': 28,
                  'axes.titlesize': 32}
    plt.rcParams.update(parameters)
    plot_parameters                  = Data()
    plot_parameters.line_width       = 3
    plot_parameters.line_styles      = ['--',':','-',':','--']
    plot_parameters.figure_width     = 10
    plot_parameters.figure_height    = 5
    plot_parameters.marker_size      = 10
    plot_parameters.legend_font_size = 20
    plot_parameters.plot_grid        = True   

    plot_parameters.colors           = [['black','firebrick','darkblue'],
                                        ['dimgray','red','blue'], 
                                        ['darkgray','salmon','deepskyblue']]  
     
    plot_parameters.markers          = ['o','v','s','P','p','^','D','X','*']   
    
        
    test_rotor_planform_function(plot_parameters)   
    return 

# ------------------------------------------------------------------ 
# Test Rotor Planform Function
# ------------------------------------------------------------------ 
def test_rotor_planform_function(PP): 
    c_r     = 0.3
    c_t     = 0.1   
    b       = 1 # span 
    r       = 11
    N       = r-1                      # number of spanwise divisions
    n       = np.linspace(N,0,r)       # vectorize
    theta_n = n*(np.pi/2)/r            # angular stations
    y_n     = b*np.cos(theta_n)        # y locations based on the angular spacing
    eta_n   = np.abs(y_n/b)            # normalized coordinates 
    p       = np.linspace(0.25,2,10)   # [0.25,0.5,1,2] 
    markers = ['s','o','P','D','v'] 
    q       = [0.25,0.5,1,1.5]         # q must be positive  
    colors  = [ cm.Greys(np.linspace(0.2,1,len(p))),
                cm.Greys(np.linspace(0.2,1,len(p))),
                cm.Greys(np.linspace(0.2,1,len(p))),
                cm.Greys(np.linspace(0.2,1,len(p)))]
    
    text_xloc_top  = [0.9,0.8,0.65,0.6] 
    text_yloc_top  = [0.3,0.3,0.26,0.25]
    text_xloc_bot  = [0.58,0.5,0.2,0.15]  
    text_yloc_bot  = [0.2,0.15,0.1,0.075] 
    
    for j in range(len(q)): 
        fig_name = "Rotor_Planform_Shape_Function_q_" + str (j)
        fig = plt.figure(fig_name)
        fig.set_size_inches(PP.figure_width,PP.figure_height)    
        axis = fig.add_subplot(1,1,1) 
        axis.set_ylabel(r'c$_n$')
        axis.set_xlabel(r'$\eta_n$')
        for i in range(len(p)):
            c_n = c_r*(1 - eta_n**p[i])**q[j] + c_t*eta_n
            line_label = 'p = ' + str(p[i]) +  ', q = ' + str(q[j]) 
            axis.plot(y_n,c_n,linestyle = '-', linewidth = PP.line_width,markersize = PP.marker_size , color = colors[j][i], label  = line_label) 
        axis.set_ylim([0.05,0.35])
        axis.text(text_xloc_top[j],text_yloc_top[j], 'p = 2.0', fontsize=25)  
        axis.text(text_xloc_bot[j],text_yloc_bot[j], 'p = 0.25', fontsize=25)      
    
        fig.tight_layout()             
        fig.savefig(fig_name  + '.png')              
    return 


if __name__ == '__main__': 
    main() 
    plt.show()