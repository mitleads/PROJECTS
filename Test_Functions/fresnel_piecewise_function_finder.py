
# imports  
import SUAVE   
from SUAVE.Optimization import Nexus, carpet_plot 
import SUAVE.Optimization.Package_Setups.scipy_setup as scipy_setup
import SUAVE.Optimization.write_optimization_outputs as write_optimization_outputs
from SUAVE.Analyses.Process import Process

# Jax Imports 
from SUAVE.Core import Units, Data
#from jax import  jit
#import jax.numpy as jnp
#from jax.numpy import where as w
#from jax.numpy import newaxis as na
#from SUAVE.Core.Utilities   import jjv, interp2d,jfresnel_sin_approx, jfresnel_cos_approx 

# python numpy and scipy 
from scipy.special  import fresnel
import numpy as np 
from scipy.special import jv 
import os , sys

# plotting 
import matplotlib.pyplot as plt  
import matplotlib.cm as cm


# main 
def main(): 
     
    
    x = np.linspace(-1,2,100) 
    

    scipy_s,scipy_c   = fresnel(x) 
    jax_s = jfresnel(x) 
    
    fig  = plt.figure('Bessel_Test')
    fig.set_size_inches(8,5)
    axis1 = fig.add_subplot(2,1,1) 
    axis2 = fig.add_subplot(2,1,2)
    axis1.plot(x,scipy_s,color =  'red', linestyle = '-', label = 'fresnel sin intergral' )
    #axis1.plot(x,scipy_c,color =   'blue', linestyle = '-', label = 'fresnel cos intergral')
    axis2.scatter(x,jax_s,color = 'red', marker = 'o', label = 'fresnel sin intergral est' )
    #axis2.scatter(x,jax_c,color = 'blue', marker = 'o')
    #axis.set_ylim(-1,0.6)
    axis1.legend(loc='best')
    axis2.legend(loc='best')
    
    return

 

def jfresnel(x):
    n = np.arange(2)  
    S = np.zeros(len(x))
    for xi in range(len(x)):
        for i in range(len(n)):
            factorial = (2*n[i] + 1) 
            arg = (-1**n[i])/(np.prod(np.arange(1,factorial+1))) * ((np.pi/2)**factorial)* ((x[xi]**(4*n[i] + 3)) / (4*n[i]+3))
            if abs(arg) == np.inf:
                break
            else: 
                S[xi] += arg   
                print(S[xi])
    return -S 
if __name__ == '__main__': 
    main() 
    plt.show()