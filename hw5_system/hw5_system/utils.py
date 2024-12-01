import math
import numpy as np
from math import copysign, fabs, sqrt, pi, sin, cos, asin, acos, atan2, exp, log


EPSILON = 1e-0 # threshold for computing quaternion from rotation matrix


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def quaternion_to_rotation_matrix(q):
        '''
        Convert a unit quaternion to a rotation matrix.

        This function is only for internal usage. Please use the following 
        instead when it is needed to do this conversion.
        
            R = q.to_rotation_matrix()

        '''
        x, y, z, w = q
        w2 = w * w
        x2 = x * x
        y2 = y * y
        z2 = z * z
        
        # Check the norm of the quaternion! It must be a unit quaternion!
        assert fabs(w2 + x2 + y2 + z2 - 1) < 1e-6
        
        wx = 2 * w * x
        wy = 2 * w * y
        wz = 2 * w * z
        
        xy = 2 * x * y
        xz = 2 * x * z
        yz = 2 * y * z
        
        R = np.asarray((
            ( w2 + x2 - y2 - z2,   xy - wz,             xz + wy           ),
            ( xy + wz,             w2 - x2 + y2 - z2,   yz - wx           ),
            ( xz - wy,             yz + wx,             w2 - x2 - y2 + z2 )
            ))
        
        return R


def rotation_matrix_to_quaternion(R):
        '''
        Convert a rotation matrix to a unit quaternion.
        
        This uses the Shepperds method for numerical stabilty.

        This function is only for internal usage. Please use the following 
        instead when it is needed to do this conversion.
        
            q = Quaternion.construct_from_rotation_matrix(R)

        '''
        
        # The rotation matrix must be orthonormal
        assert np.allclose(np.dot(R, R.conj().transpose()), np.eye(3), 
                           atol=EPSILON)
    
        # Check the determinant of R! It must be 1.
        assert math.isclose(np.linalg.det(R), 1, abs_tol=EPSILON)
        
        w2 = (1 + R[0, 0] + R[1, 1] + R[2, 2])
        x2 = (1 + R[0, 0] - R[1, 1] - R[2, 2])
        y2 = (1 - R[0, 0] + R[1, 1] - R[2, 2])
        z2 = (1 - R[0, 0] - R[1, 1] + R[2, 2])
            
        yz = (R[1, 2] + R[2, 1])
        xz = (R[2, 0] + R[0, 2])
        xy = (R[0, 1] + R[1, 0])
    
        wx = (R[2, 1] - R[1, 2])
        wy = (R[0, 2] - R[2, 0])
        wz = (R[1, 0] - R[0, 1])
                    
            
        if R[2, 2] < 0:
          
            if R[0, 0] > R[1, 1]:
            
                x = sqrt(x2)
                w = wx / x
                y = xy / x
                z = xz / x
            
            else:
                 
                y = sqrt(y2)
                w = wy / y
                x = xy / y
                z = yz / y
    
        else:
              
            if R[0, 0] < -R[1, 1]:
                 
                z = sqrt(z2)
                w = wz / z
                x = xz / z
                y = yz / z
            
            else:
                 
                w = sqrt(w2)
                x = wx / w
                y = wy / w
                z = wz / w
        
        w = w * 0.5
        x = x * 0.5
        y = y * 0.5
        z = z * 0.5
        
        return x, y, z, w

def combine_transformations(R1, t1, R2, t2):
    """
    Combine two transformations given by rotation matrices and translation vectors.
    
    Parameters:
    R1 (numpy.ndarray): First 3x3 rotation matrix.
    t1 (numpy.ndarray): First translation vector, shape (3,).
    R2 (numpy.ndarray): Second 3x3 rotation matrix.
    t2 (numpy.ndarray): Second translation vector, shape (3,).
    
    Returns:
    R_combined (numpy.ndarray): Combined 3x3 rotation matrix.
    t_combined (numpy.ndarray): Combined translation vector, shape (3,).
    """
    # Ensure the input matrices and vectors have correct shapes
    assert R1.shape == (3, 3), "R1 must be a 3x3 matrix"
    assert t1.shape == (3,), "t1 must be a 3-element vector"
    assert R2.shape == (3, 3), "R2 must be a 3x3 matrix"
    assert t2.shape == (3,), "t2 must be a 3-element vector"
    
    T1 = np.eye(4)
    T1[:3, :3] = R1
    T1[:3, 3] = t1

    T2 = np.eye(4)
    T2[:3, :3] = R2
    T2[:3, 3] = t2
    
    return np.dot(T1, T2)
