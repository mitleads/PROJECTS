
# imports 

# Jax Imports 
from SUAVE.Core import to_jnumpy
from jax import  jit
import jax.numpy as jnp
from jax.numpy import where as w
from jax.numpy import newaxis as na
from SUAVE.Core.Utilities   import jjv, interp2d,jfresnel_sin_approx, jfresnel_cos_approx 

# python numpy and scipy 
from scipy.special  import fresnel
import numpy as np 
from scipy.special import jv 

# plotting 
import matplotlib.pyplot as plt  
import matplotlib.cm as cm

# main 
def main(): 
    
    #bessel_function_test()
    fresnel_function_test()
    
    return 

def bessel_function_test():
    m = np.arange(1,5)
    x = np.linspace(0,10,100)
    
    colors  = cm.viridis(np.linspace(0,1,len(m)))  
    M = np.tile(m[:,None],(1,len(x)))
    X = np.tile(x[None,:],(len(m),1))
    
    scipy_jv = jv(M,X)
    jax_jv   = jjv(M,X)
    
    fig  = plt.figure('Bessel_Test')
    fig.set_size_inches(8,5)
    axis = fig.add_subplot(1,1,1)
    for i in range(len(m)): 
        axis.plot(x,scipy_jv[i],color = colors[i], linestyle = '-', label = 'J = ' + str(i+1))
        axis.scatter(x,jax_jv[i],color = colors[i], marker = 'o')
    axis.set_ylim(-0.4,0.6)
    axis.legend(loc='best')
    return 

def fresnel_function_test(): 
    x = np.linspace(-10,10,1000) 
    
    scipy_s,scipy_c = fresnel(x)
    jax_s           = jfresnel_sin_approx(x)
    jax_c           = jfresnel_cos_approx(x) 
    
    fig  = plt.figure('Bessel_Test')
    fig.set_size_inches(8,5)
    axis1 = fig.add_subplot(2,1,1) 
    axis2 = fig.add_subplot(2,1,2)
    axis1.plot(x,scipy_s,color =  'red', linestyle = '-', label = 'fresnel sin intergral' )
    axis2.plot(x,scipy_c,color =   'blue', linestyle = '-', label = 'fresnel cos intergral')
    #axis1.scatter(x,jax_s,color = 'red', marker = 'o')
    #axis2.scatter(x,jax_c,color = 'blue', marker = 'o')
    #axis.set_ylim(-1,0.6)
    axis1.legend(loc='best')
    axis2.legend(loc='best')
    return 



if __name__ == '__main__': 
    main() 
    plt.show()