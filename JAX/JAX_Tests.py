# JAX TEXTS

from jax import jit, grad
import jax.numpy as jnp
import numpy as np
import timeit
import time 
 
from test_set_baseline import *
from test_set_noise    import *
'''The follow tests are meant to verify the functionality of JAX in SUAVE'''

def main():
    # compute_tests()
    noise_test()
    
    return 
# ----------------------------------------------------------------------
#   JAX BASELINE TESTS
# ----------------------------------------------------------------------
def compute_tests():
    
    baseline_test_single()
    baseline_test_small()
    baseline_test_medium() 
    baseline_test_large()  
    
    '''
    Results 
    
    Singel Numpy Test time taken    : 6.431098222732544 sec
    Single Jax Numpy Test time taken: 1.822721004486084 sec
    Single Test Speed-up Ratio     : 3.528295447797175
    Small Numpy time taken    : 27.429414749145508 sec
    Small Jax Numpy time taken: 7.369951009750366 sec
    Small Test Speed-up Ratio: 3.721790648656509
    Medium Numpy Test time taken    : 169.80474305152893 sec
    Medium Jax Numpy Test time taken: 47.1646990776062 sec
    Medium Test Speed-up Ratio: 3.6002507462652766
    Large Numpy Test time taken    : 650.2098228931427 sec
    Large Jax Numpy Test time taken: 150.62279200553894 sec
    Large Test Speed-up Ratio: 4.316808991757583
    ''' 
    return 


# ----------------------------------------------------------------------
#   Noise Analysis Test 
# ----------------------------------------------------------------------
def noise_test():


    return 

# ----------------------------------------------------------------------
#   VLM Anakysis Test
# ----------------------------------------------------------------------
def VLM_analysis_test():


    return 





# ----------------------------------------------------------------------
#   Battery Energy Analysis Test
# ----------------------------------------------------------------------
def Battery_energy_analysis_test():


    return 
if __name__ == '__main__': 
    main()    
    plt.show()




