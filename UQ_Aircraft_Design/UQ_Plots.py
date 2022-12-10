  
from SUAVE.Core import Data   
import matplotlib.pyplot as plt
import numpy as np  
import matplotlib.cm as cm 
   

def plot_uq_results(UQ_RESULTS): 
    sensitivity="first"
    sensitivity, _ = convert_sensitivity(sensitivity)

    plt.rcParams['axes.linewidth'] = 2.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 20,
                  'xtick.labelsize': 20,
                  'ytick.labelsize': 20,
                  'axes.titlesize': 20}
    plt.rcParams.update(parameters)    
    plot_parameters                  = Data()  
    plot_parameters.linestyles       = '-'
    plot_parameters.markers          = ['o','v','s','P','p','^','D','X','*']
    plot_parameters.colors           = ['black','red','mediumblue','darkgreen', 'darkorange' ]   
    plot_parameters.marker_size      = 10 
    plot_parameters.legend_font_size = 14 
    plot_parameters.labelsize        = 20
    plot_parameters.titlesize        = 20
    plot_parameters.linewidth        = 2
    plot_parameters.fontsize         = 20 
    plot_parameters.fig_width        = 8
    plot_parameters.fig_height       = 4 
     
    for i in range(len(list(UQ_RESULTS.keys()))):
        res_name = list(UQ_RESULTS.keys())[i]
        uq_res = UQ_RESULTS[res_name]
        for feature in uq_res.data:
            if uq_res[feature].ndim() == 1:   
                plot_mean_variance_1d(uq_res,feature, plot_parameters)
                plot_prediction_interval_1d(uq_res,feature, plot_parameters)
    
                if sensitivity in uq_res.data[feature]:
                    plot_sensitivity_1d_grid(uq_res,feature, sensitivity, plot_parameters) 
    return 

def plot_mean_variance_1d(uq_res,feature,plot_parameters): 

    if feature is None:
        feature = uq_res.data.model_name

    if uq_res[feature].ndim() != 1:
        raise ValueError("{} is not a 1D feature".format(feature))
 

    if uq_res.data[feature].time is None or np.all(np.isnan(uq_res.data[feature].time)):
        time = np.arange(0, len(uq_res.data[feature].mean))
    else:
        time = uq_res.data[feature].time


    labels = get_labels(uq_res.data,feature)
    xlabel, ylabel = labels 

    title    = feature + ", mean and variance"
    fig_name = title.replace("_", " ")
    fig      = plt.figure(fig_name) 
    fig.set_size_inches(plot_parameters.fig_width,plot_parameters.fig_height)   
    ax       = fig.add_subplot(1,1,1)
    ax.plot(time, uq_res.data[feature].mean,color = plot_parameters.colors[0],
                  linestyle = plot_parameters.linestyles,  linewidth=plot_parameters.linewidth)
    ax.tick_params(axis="y", color=plot_parameters.colors[0], labelcolor=plot_parameters.colors[0])
    ax.spines["left"].set_edgecolor(plot_parameters.colors[0])
    ax.set_ylabel(r"$\mu$", color=plot_parameters.colors[0],fontsize=plot_parameters.labelsize)
    ax.set_xlabel(xlabel) 
    ax.set_xlim([min(time), max(time)])
  
    ax2     = ax.twinx() 
    ax2.tick_params(axis="y", which="both", right=False, left=False, labelright=True,
                    color=plot_parameters.colors[1], labelcolor=plot_parameters.colors[1], labelsize=plot_parameters.labelsize)
    ax2.set_ylabel(r"$\sigma^2$" ,color=plot_parameters.colors[1], fontsize=plot_parameters.labelsize) 
    ax2.plot(time, uq_res.data[feature].variance,
             color=plot_parameters.colors[1], linestyle = plot_parameters.linestyles,
             linewidth=plot_parameters.linewidth, antialiased=True) 
    ax2.yaxis.offsetText.set_fontsize(plot_parameters.fontsize)
    ax2.yaxis.offsetText.set_color(plot_parameters.colors[1]) 
    ax2.spines["right"].set_visible(True)
    ax2.spines["right"].set_edgecolor(plot_parameters.colors[1]) 
    ax2.set_xlim([min(time), max(time)])


    plt.tight_layout() 
    
    return 


def plot_prediction_interval_1d(uq_res,feature,plot_parameters):  

    if uq_res.data[feature].time is None or np.all(np.isnan(uq_res.data[feature].time)):
        time = np.arange(0, len(uq_res.data[feature].mean))
    else:
        time = uq_res.data[feature].time 

    labels         = get_labels(uq_res.data,feature)
    xlabel, ylabel = labels 

    title    = feature +  ", 95% prediction interval"
    fig_name = title.replace("_", " ")
    fig      = plt.figure(fig_name) 
    fig.set_size_inches(plot_parameters.fig_width,plot_parameters.fig_height)   
    ax       = fig.add_subplot(1,1,1)
    ax.set_ylabel(r"$" + ylabel + "$")
    ax.set_xlabel(xlabel)
    ax.fill_between(time, uq_res.data[feature].percentile_5, uq_res.data[feature].percentile_95, alpha=0.5, color=plot_parameters.colors[0],  linewidth=0) 
    ax.plot(time, uq_res.data[feature].mean,color = plot_parameters.colors[0], linewidth=plot_parameters.linewidth)  
    ax.set_xlim([min(time), max(time)])
    plt.legend(["Mean", "95% Prediction Interval"], loc="best", prop={'size':  plot_parameters.legend_font_size}) 
    plt.tight_layout() 

    return 
 



def plot_sensitivity_1d_grid(uq_res,feature,sensitivity,plot_parameters): 
    
    sensitivity, title = convert_sensitivity(sensitivity) 
    labels             = get_labels(uq_res.data,feature)
    xlabel, ylabel     = labels
    parameter_names    = uq_res.uncertain_parameters

    if uq_res.data[feature].time is None or np.all(np.isnan(uq_res.data[feature].time)):
        time = np.arange(0, len(uq_res.data[feature][sensitivity][0]))
    else:
        time = uq_res.data[feature].time
 

    fig  = plt.figure("First_Order_Sobol_Indices") 
    fig.set_size_inches(plot_parameters.fig_width,plot_parameters.fig_height)   
    axes = fig.add_subplot(1,1,1)  
    axes.set_ylim([0, 1.05])
    axes.set_xlim([min(time), max(time)])   
    axes.set_xlabel(xlabel)
    axes.set_ylabel(title.capitalize()) 
    
    for i in range( len(parameter_names)): 
        try:
            indice_label = r"$" + parameter_names[i].split("_")[0] + '{' + parameter_names[i].split("_")[1] + '}' + "$"
        except:
            indice_label = r"$" + parameter_names[i] + "$"
        axes.plot(time, uq_res.data[feature][sensitivity][i],color=plot_parameters.colors[i],marker = plot_parameters.markers[i], linestyle = plot_parameters.linestyles, linewidth=plot_parameters.linewidth, label = indice_label) 
    plt.legend(loc="best", prop={'size':  plot_parameters.legend_font_size})  
    plt.tight_layout()  
    return 


def convert_sensitivity(sensitivity):
    if sensitivity == "first":
        sensitivity = "sobol_first"
    elif sensitivity == "total":
        sensitivity = "sobol_total"

    full_text = ""
    if sensitivity == "sobol_first":
        full_text = "first order Sobol indices"
    elif sensitivity == "sobol_total":
        full_text = "total order Sobol indices"

    return sensitivity, full_text


def get_labels(uq_res, feature): 
    if uq_res[feature].labels != []:
        return uq_res[feature].labels

    elif uq_res[uq_res.model_name].labels != [] and uq_res[uq_res.model_name].ndim() == uq_res[feature].ndim():
        return uq_res[uq_res.model_name].labels

    else:
        return [""]*(uq_res[feature].ndim() + 1)
    
 