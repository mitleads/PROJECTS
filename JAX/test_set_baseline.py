
from jax import jit, grad
import jax.numpy as jnp
import numpy as np
import timeit
import time 

'''The follow tests are meant to verify the functionality of JAX in SUAVE'''
 

# ----------------------------------------------------------------------
#   JAX TIME TEST
# ----------------------------------------------------------------------
def baseline_test_single():
     

    ti = time.time() 
  
    N = 10000
    A1        = np.random.random((N,N))
    B1        = np.random.random((N,N))
    C1        = np.random.random((N,N)) 
    np_res1   = np_function(A1,B1,C1) 
    
    tf = time.time()
    print ('Singel Numpy Test time taken    : ' + str(tf-ti)  + ' sec')       
    

    ti2       = time.time()  
    
    A2       = jnp.array(A1)
    B2       = jnp.array(B1)
    C2       = jnp.array(C1)
    jnp_res1  = jnp_function(A2,B2,C2) 

    tf2 = time.time()
    print ('Single Jax Numpy Test time taken: ' + str(tf2-ti2)  + ' sec')
    
    time_ratio = (tf-ti)/(tf2-ti2)
    print ('Single Test Speed-up Ratio     : ' + str(time_ratio))
    return 

def baseline_test_small():
     

    ti = time.time() 
  
    N = 10000
    A1        = np.random.random((N,N))
    B1        = np.random.random((N,N))
    C1        = np.random.random((N,N)) 
    np_res1   = np_function(A1,B1,C1)
    np_res2   = np_function(A1,B1,C1)
    np_res3   = np_function(A1,B1,C1)
    np_res4   = np_function(A1,B1,C1)
    np_res5   = np_function(A1,B1,C1) 
    
    tf = time.time()
    print ('Small Numpy time taken    : ' + str(tf-ti)  + ' sec')       
    

    ti2       = time.time()  
    
    A2       = jnp.array(A1)
    B2       = jnp.array(B1)
    C2       = jnp.array(C1)
    jnp_res1  = jnp_function(A2,B2,C2)
    jnp_res2  = jnp_function(A2,B2,C2)
    jnp_res3  = jnp_function(A2,B2,C2)
    jnp_res4  = jnp_function(A2,B2,C2)
    jnp_res5  = jnp_function(A2,B2,C2) 

    tf2 = time.time()
    print ('Small Jax Numpy time taken: ' + str(tf2-ti2)  + ' sec')
    

    time_ratio = (tf-ti)/(tf2-ti2)
    print ('Small Test Speed-up Ratio: ' + str(time_ratio))    
    
    return 

def baseline_test_medium():

    ti = time.time() 
  
    N = 10000
    A1        = np.random.random((N,N))
    B1        = np.random.random((N,N))
    C1        = np.random.random((N,N)) 
    np_res1   = np_function(A1,B1,C1)
    np_res2   = np_function(A1,B1,C1)
    np_res3   = np_function(A1,B1,C1)
    np_res4   = np_function(A1,B1,C1)
    np_res5   = np_function(A1,B1,C1)
    np_res6   = np_function(A1,B1,C1)
    np_res7   = np_function(A1,B1,C1)
    np_res8   = np_function(A1,B1,C1)
    np_res9   = np_function(A1,B1,C1)
    np_res10  = np_function(A1,B1,C1)
    np_res11  = np_function(A1,B1,C1)
    np_res12  = np_function(A1,B1,C1)
    np_res13  = np_function(A1,B1,C1) 
    np_res14   = np_function(A1,B1,C1)
    np_res15   = np_function(A1,B1,C1)
    np_res16   = np_function(A1,B1,C1)
    np_res17   = np_function(A1,B1,C1)
    np_res18   = np_function(A1,B1,C1)
    np_res19   = np_function(A1,B1,C1)
    np_res20   = np_function(A1,B1,C1) 
    np_res21   = np_function(A1,B1,C1) 
    np_res22  = np_function(A1,B1,C1)
    np_res23  = np_function(A1,B1,C1)
    np_res24  = np_function(A1,B1,C1)
    np_res25  = np_function(A1,B1,C1)
    np_res26  = np_function(A1,B1,C1)
    np_res27  = np_function(A1,B1,C1)
    np_res28  = np_function(A1,B1,C1)
    np_res29  = np_function(A1,B1,C1) 
    np_res30  = np_function(A1,B1,C1) 
    
    tf = time.time()
    print ('Medium Numpy Test time taken    : ' + str(tf-ti)  + ' sec')       
    

    ti2       = time.time()  
    
    A2       = jnp.array(A1)
    B2       = jnp.array(B1)
    C2       = jnp.array(C1)
    jnp_res1   = np_function(A2,B2,C2)
    jnp_res2  = jnp_function(A2,B2,C2)
    jnp_res3  = jnp_function(A2,B2,C2)
    jnp_res4  = jnp_function(A2,B2,C2)
    jnp_res5  = jnp_function(A2,B2,C2)
    jnp_res6  = jnp_function(A2,B2,C2)
    jnp_res7  = jnp_function(A2,B2,C2)
    jnp_res8  = jnp_function(A2,B2,C2)
    jnp_res9  = jnp_function(A2,B2,C2)
    jnp_res10 = jnp_function(A2,B2,C2)
    jnp_res11 = jnp_function(A2,B2,C2)
    jnp_res12 = jnp_function(A2,B2,C2)
    jnp_res13 = jnp_function(A2,B2,C2)   
    jnp_res14  = jnp_function(A2,B2,C2)
    jnp_res15  = jnp_function(A2,B2,C2)
    jnp_res16  = jnp_function(A2,B2,C2)
    jnp_res17  = jnp_function(A2,B2,C2)
    jnp_res18  = jnp_function(A2,B2,C2)
    jnp_res19  = jnp_function(A2,B2,C2)
    jnp_res20  = jnp_function(A2,B2,C2) 
    jnp_res21  = jnp_function(A2,B2,C2) 
    jnp_res22  = jnp_function(A2,B2,C2)
    jnp_res23  = jnp_function(A2,B2,C2)
    jnp_res24  = jnp_function(A2,B2,C2)
    jnp_res25  = jnp_function(A2,B2,C2)
    jnp_res26  = jnp_function(A2,B2,C2)
    jnp_res27  = jnp_function(A2,B2,C2)
    jnp_res28  = jnp_function(A2,B2,C2)
    jnp_res29  = jnp_function(A2,B2,C2) 
    jnp_res30  = jnp_function(A2,B2,C2) 
    

    tf2 = time.time()
    print ('Medium Jax Numpy Test time taken: ' + str(tf2-ti2)  + ' sec')
    

    time_ratio = (tf-ti)/(tf2-ti2)
    print ('Medium Test Speed-up Ratio: ' + str(time_ratio))    
    
    return 


def baseline_test_large():

    ti = time.time() 
  
    N = 10000
    A1        = np.random.random((N,N))
    B1        = np.random.random((N,N))
    C1        = np.random.random((N,N)) 
    np_res1   = np_function(A1,B1,C1)
    np_res2   = np_function(A1,B1,C1)
    np_res3   = np_function(A1,B1,C1)
    np_res4   = np_function(A1,B1,C1)
    np_res5   = np_function(A1,B1,C1)
    np_res6   = np_function(A1,B1,C1)
    np_res7   = np_function(A1,B1,C1)
    np_res8   = np_function(A1,B1,C1)
    np_res9   = np_function(A1,B1,C1)
    np_res10  = np_function(A1,B1,C1)
    np_res11  = np_function(A1,B1,C1)
    np_res12  = np_function(A1,B1,C1)
    np_res13  = np_function(A1,B1,C1) 
    np_res14   = np_function(A1,B1,C1)
    np_res15   = np_function(A1,B1,C1)
    np_res16   = np_function(A1,B1,C1)
    np_res17   = np_function(A1,B1,C1)
    np_res18   = np_function(A1,B1,C1)
    np_res19   = np_function(A1,B1,C1)
    np_res20   = np_function(A1,B1,C1) 
    np_res21   = np_function(A1,B1,C1) 
    np_res22  = np_function(A1,B1,C1)
    np_res23  = np_function(A1,B1,C1)
    np_res24  = np_function(A1,B1,C1)
    np_res25  = np_function(A1,B1,C1)
    np_res26  = np_function(A1,B1,C1)
    np_res27  = np_function(A1,B1,C1)
    np_res28  = np_function(A1,B1,C1)
    np_res29  = np_function(A1,B1,C1) 
    np_res30  = np_function(A1,B1,C1)  
    np_res31  = np_function(A1,B1,C1) 
    np_res32  = np_function(A1,B1,C1)
    np_res33  = np_function(A1,B1,C1)
    np_res34  = np_function(A1,B1,C1)
    np_res35  = np_function(A1,B1,C1)
    np_res36  = np_function(A1,B1,C1)
    np_res37  = np_function(A1,B1,C1)
    np_res38  = np_function(A1,B1,C1)
    np_res39  = np_function(A1,B1,C1) 
    np_res40  = np_function(A1,B1,C1) 
    np_res41  = np_function(A1,B1,C1) 
    np_res42  = np_function(A1,B1,C1)
    np_res43  = np_function(A1,B1,C1)
    np_res44  = np_function(A1,B1,C1)
    np_res45  = np_function(A1,B1,C1)
    np_res46  = np_function(A1,B1,C1)
    np_res47  = np_function(A1,B1,C1)
    np_res48  = np_function(A1,B1,C1)
    np_res49  = np_function(A1,B1,C1) 
    np_res50  = np_function(A1,B1,C1) 
    np_res51  = np_function(A1,B1,C1) 
    np_res52  = np_function(A1,B1,C1)
    np_res53  = np_function(A1,B1,C1)
    np_res54  = np_function(A1,B1,C1)
    np_res55  = np_function(A1,B1,C1)
    np_res56  = np_function(A1,B1,C1)
    np_res57  = np_function(A1,B1,C1)
    np_res58  = np_function(A1,B1,C1)
    np_res59  = np_function(A1,B1,C1) 
    np_res60  = np_function(A1,B1,C1) 
    np_res61  = np_function(A1,B1,C1) 
    np_res62  = np_function(A1,B1,C1)
    np_res63  = np_function(A1,B1,C1)
    np_res64  = np_function(A1,B1,C1)
    np_res65  = np_function(A1,B1,C1)
    np_res66  = np_function(A1,B1,C1)
    np_res67  = np_function(A1,B1,C1)
    np_res68  = np_function(A1,B1,C1)
    np_res69  = np_function(A1,B1,C1) 
    np_res70  = np_function(A1,B1,C1) 
    np_res71  = np_function(A1,B1,C1) 
    np_res72  = np_function(A1,B1,C1)
    np_res73  = np_function(A1,B1,C1)
    np_res74  = np_function(A1,B1,C1)
    np_res75  = np_function(A1,B1,C1)
    np_res76  = np_function(A1,B1,C1)
    np_res77  = np_function(A1,B1,C1)
    np_res78  = np_function(A1,B1,C1)
    np_res79  = np_function(A1,B1,C1) 
    np_res80  = np_function(A1,B1,C1) 
    np_res81  = np_function(A1,B1,C1) 
    np_res82  = np_function(A1,B1,C1)
    np_res83  = np_function(A1,B1,C1)
    np_res84  = np_function(A1,B1,C1)
    np_res85  = np_function(A1,B1,C1)
    np_res86  = np_function(A1,B1,C1)
    np_res87  = np_function(A1,B1,C1)
    np_res88  = np_function(A1,B1,C1)
    np_res89  = np_function(A1,B1,C1) 
    np_res90  = np_function(A1,B1,C1) 
    np_res91  = np_function(A1,B1,C1) 
    np_res92  = np_function(A1,B1,C1)
    np_res93  = np_function(A1,B1,C1)
    np_res94  = np_function(A1,B1,C1)
    np_res95  = np_function(A1,B1,C1)
    np_res96  = np_function(A1,B1,C1)
    np_res97  = np_function(A1,B1,C1)
    np_res98  = np_function(A1,B1,C1)
    np_res99  = np_function(A1,B1,C1) 
    np_res100 = np_function(A1,B1,C1)  
    
    tf = time.time()
    print ('Large Numpy Test time taken    : ' + str(tf-ti)  + ' sec')       
    

    ti2       = time.time()  
    
    A2       = jnp.array(A1)
    B2       = jnp.array(B1)
    C2       = jnp.array(C1)
    jnp_res1   = np_function(A2,B2,C2)
    jnp_res2  = jnp_function(A2,B2,C2)
    jnp_res3  = jnp_function(A2,B2,C2)
    jnp_res4  = jnp_function(A2,B2,C2)
    jnp_res5  = jnp_function(A2,B2,C2)
    jnp_res6  = jnp_function(A2,B2,C2)
    jnp_res7  = jnp_function(A2,B2,C2)
    jnp_res8  = jnp_function(A2,B2,C2)
    jnp_res9  = jnp_function(A2,B2,C2)
    jnp_res10 = jnp_function(A2,B2,C2)
    jnp_res11 = jnp_function(A2,B2,C2)
    jnp_res12 = jnp_function(A2,B2,C2)
    jnp_res13 = jnp_function(A2,B2,C2)   
    jnp_res14  = jnp_function(A2,B2,C2)
    jnp_res15  = jnp_function(A2,B2,C2)
    jnp_res16  = jnp_function(A2,B2,C2)
    jnp_res17  = jnp_function(A2,B2,C2)
    jnp_res18  = jnp_function(A2,B2,C2)
    jnp_res19  = jnp_function(A2,B2,C2)
    jnp_res20  = jnp_function(A2,B2,C2) 
    jnp_res21  = jnp_function(A2,B2,C2) 
    jnp_res22  = jnp_function(A2,B2,C2)
    jnp_res23  = jnp_function(A2,B2,C2)
    jnp_res24  = jnp_function(A2,B2,C2)
    jnp_res25  = jnp_function(A2,B2,C2)
    jnp_res26  = jnp_function(A2,B2,C2)
    jnp_res27  = jnp_function(A2,B2,C2)
    jnp_res28  = jnp_function(A2,B2,C2)
    jnp_res29  = jnp_function(A2,B2,C2) 
    jnp_res30  = jnp_function(A2,B2,C2) 
    jnp_res31  = jnp_function(A2,B2,C2) 
    jnp_res32  = jnp_function(A2,B2,C2)
    jnp_res33  = jnp_function(A2,B2,C2)
    jnp_res34  = jnp_function(A2,B2,C2)
    jnp_res35  = jnp_function(A2,B2,C2)
    jnp_res36  = jnp_function(A2,B2,C2)
    jnp_res37  = jnp_function(A2,B2,C2)
    jnp_res38  = jnp_function(A2,B2,C2)
    jnp_res39  = jnp_function(A2,B2,C2) 
    jnp_res40  = jnp_function(A2,B2,C2) 
    jnp_res41  = jnp_function(A2,B2,C2) 
    jnp_res42  = jnp_function(A2,B2,C2)
    jnp_res43  = jnp_function(A2,B2,C2)
    jnp_res44  = jnp_function(A2,B2,C2)
    jnp_res45  = jnp_function(A2,B2,C2)
    jnp_res46  = jnp_function(A2,B2,C2)
    jnp_res47  = jnp_function(A2,B2,C2)
    jnp_res48  = jnp_function(A2,B2,C2)
    jnp_res49  = jnp_function(A2,B2,C2) 
    jnp_res50  = jnp_function(A2,B2,C2) 
    jnp_res51  = jnp_function(A2,B2,C2) 
    jnp_res52  = jnp_function(A2,B2,C2)
    jnp_res53  = jnp_function(A2,B2,C2)
    jnp_res54  = jnp_function(A2,B2,C2)
    jnp_res55  = jnp_function(A2,B2,C2)
    jnp_res56  = jnp_function(A2,B2,C2)
    jnp_res57  = jnp_function(A2,B2,C2)
    jnp_res58  = jnp_function(A2,B2,C2)
    jnp_res59  = jnp_function(A2,B2,C2) 
    jnp_res60  = jnp_function(A2,B2,C2) 
    jnp_res61  = jnp_function(A2,B2,C2) 
    jnp_res62  = jnp_function(A2,B2,C2)
    jnp_res63  = jnp_function(A2,B2,C2)
    jnp_res64  = jnp_function(A2,B2,C2)
    jnp_res65  = jnp_function(A2,B2,C2)
    jnp_res66  = jnp_function(A2,B2,C2)
    jnp_res67  = jnp_function(A2,B2,C2)
    jnp_res68  = jnp_function(A2,B2,C2)
    jnp_res69  = jnp_function(A2,B2,C2) 
    jnp_res70  = jnp_function(A2,B2,C2) 
    jnp_res71  = jnp_function(A2,B2,C2) 
    jnp_res72  = jnp_function(A2,B2,C2)
    jnp_res73  = jnp_function(A2,B2,C2)
    jnp_res74  = jnp_function(A2,B2,C2)
    jnp_res75  = jnp_function(A2,B2,C2)
    jnp_res76  = jnp_function(A2,B2,C2)
    jnp_res77  = jnp_function(A2,B2,C2)
    jnp_res78  = jnp_function(A2,B2,C2)
    jnp_res79  = jnp_function(A2,B2,C2) 
    jnp_res80  = jnp_function(A2,B2,C2) 
    jnp_res81  = jnp_function(A2,B2,C2) 
    jnp_res82  = jnp_function(A2,B2,C2)
    jnp_res83  = jnp_function(A2,B2,C2)
    jnp_res84  = jnp_function(A2,B2,C2)
    jnp_res85  = jnp_function(A2,B2,C2)
    jnp_res86  = jnp_function(A2,B2,C2)
    jnp_res87  = jnp_function(A2,B2,C2)
    jnp_res88  = jnp_function(A2,B2,C2)
    jnp_res89  = jnp_function(A2,B2,C2) 
    jnp_res90  = jnp_function(A2,B2,C2) 
    jnp_res91  = jnp_function(A2,B2,C2) 
    jnp_res92  = jnp_function(A2,B2,C2)
    jnp_res93  = jnp_function(A2,B2,C2)
    jnp_res94  = jnp_function(A2,B2,C2)
    jnp_res95  = jnp_function(A2,B2,C2)
    jnp_res96  = jnp_function(A2,B2,C2)
    jnp_res97  = jnp_function(A2,B2,C2)
    jnp_res98  = jnp_function(A2,B2,C2)
    jnp_res99  = jnp_function(A2,B2,C2) 
    jnp_res100  = jnp_function(A2,B2,C2)  

    tf2 = time.time()
    print ('Large Jax Numpy Test time taken: ' + str(tf2-ti2)  + ' sec')
    

    time_ratio = (tf-ti)/(tf2-ti2)
    print ('Large Test Speed-up Ratio: ' + str(time_ratio))    
    
    return 


@jit
@grad
def jnp_function(A,B,C):
    step_1 = jnp.cos(A)
    step_2 = step_1/B
    step_3 = jnp.ones_like(step_1)
    step_4 = jnp.tan(step_2)/jnp.cos(C)
    step_5 = (step_3 + step_4)/5.
    step_6 = step_5 + step_4
    return jnp.sum(step_6)

def np_function(A,B,C):
    step_1 = np.cos(A)
    step_2 = step_1/B
    step_3 = np.ones_like(step_1)
    step_4 = np.tan(step_2)/np.cos(C)
    step_5 = (step_3 + step_4)/5.
    step_6 = step_5 + step_4

    return np.sum(step_6) 


