from tensorflow_probability.substrates import jax as tfp
import tensorflow as tf
from scipy.special import jv, jve, iv, ive, fresnel
import  numpy as np
import jax.numpy as jnp
from tensorflow.python.ops.special_math_ops import fresnel_sin, fresnel_cos
from jax.experimental import jax2tf
def main():
    v = np.ones(4,dtype=np.float32)*4.
    z = np.ones(4,dtype=np.float32)*2.
    print('scipy Bessel' )
    print(iv(v,z))
    print('tfp Bessel')
    print(tfp.math.bessel_ive(v,z)/np.exp(-abs(z)))
    print('scipy fresnel')
    print(fresnel(z))
    print('tf fresnel sin')
    print(tf.math.special.fresnel_sin(z))
    print('tf fresnel cos')
    print(tf.math.special.fresnel_cos(z))
    print('JAX TF')
    fs, fc = jax2tf.call_tf(fresnel_tf)(z)
    print(fs)
    print(fc)
    
def fresnel_tf(z):
    return fresnel_sin(z), fresnel_cos(z) 
 
if __name__ == '__main__':
    main()