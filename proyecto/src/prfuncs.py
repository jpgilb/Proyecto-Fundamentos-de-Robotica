import numpy as np
from copy import copy
from pyquaternion import Quaternion
import rbdl

pi = np.pi

class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('/home/user/proy_ws/src/robot_description/urdf/robot_description.urdf')

    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq

# Matriz de transformación homogénea a partir de parámetros de Denavit-Hartenberg
def dh(d, theta, a, alpha):
 sth = np.sin(theta)
 cth = np.cos(theta)
 sa  = np.sin(alpha)
 ca  = np.cos(alpha)
 T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
               [sth,  ca*cth, -sa*cth, a*sth],
               [0.0,      sa,      ca,     d],
               [0.0,     0.0,     0.0,   1.0]])
 return T


# Cinemática directa de un robot de 6GDL a partir de transformaciones de DH
def fkine(q):
 T1 = dh(0.3231,   pi/2+q[0],  0.0,    pi/2)
 T2 = dh(0.0,      pi/2+q[1],  0.2105, 0.0)
 T3 = dh(0.0,      pi/2+q[2],  0.0,    pi/2)
 T4 = dh(q[3],     0.0,        0.0,    0.0)
 T5 = dh(0.4263,   pi+q[4],    0.0,    pi/2)
 T6 = dh(0.0,      pi+q[5],    0.0,    pi/2)
 T7 = dh(0.1636,   q[6],       0.0,    0.0)
 T = T1 @ T2 @ T3 @ T4 @ T5 @ T6 @ T7
 return T

def TF2xyzquat(T):
 quat = Quaternion(matrix=T[0:3,0:3])
 return np.array([T[0,3], T[1,3], T[2,3], quat.w, quat.x, quat.y, quat.z])

def jacobian(q, delta=0.0001):
 """
 Jacobiano analitico para la posicion de un brazo robotico de n grados de libertad. 
 Retorna una matriz de 3xn y toma como entrada el vector de configuracion articular 
 q=[q1, q2, q3, ..., qn]
 """
 # Crear una matriz 3xn
 n = q.size
 J = np.zeros((3,n))
 # Calcular la transformacion homogenea inicial (usando q)
 T = fkine(q)
 # Iteracion para la derivada de cada articulacion (columna)
 for i in range(n):
  # Copiar la configuracion articular inicial
  dq = copy(q)
  # Calcular nuevamenta la transformacion homogenea e
  # Incrementar la articulacion i-esima usando un delta
  dq[i] += delta
  # Transformacion homogenea luego del incremento (q+delta)
  T_inc = fkine(dq)
  # Aproximacion del Jacobiano de posicion usando diferencias finitas
  J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
 return J



def jacobian_pose(q, delta=0.0001):
 """
 Jacobiano analitico para la posicion y orientacion (usando un
 cuaternion). Retorna una matriz de 7xn y toma como entrada el vector de
 configuracion articular q=[q1, q2, q3, ..., qn]
 """
 n = q.size
 J = np.zeros((7,n))

 # Pose inicial
 T = fkine(q)
 Pose = TF2xyzquat(T)

 for i in range(n):
  # Copiar configuración incial
  dq = copy(q)
  # Incrementar articulación i-ésima
  dq[i] += delta
  # Pose luego del incremento
  T_inc = fkine(dq)
  Pose_inc = TF2xyzquat(T_inc)
  # Aproximación del Jacobiano
  J[0:7, i] = (Pose_inc - Pose)/delta  
 return J

def PoseError(x,xd):
 """
 Determine the pose error of the end effector.

 Input:
 x -- Actual position of the end effector, in the format [x y z ew ex ey ez]
 xd -- Desire position of the end effector, in the format [x y z ew ex ey ez]
 Output:
 err_pose -- Error position of the end effector, in the format [x y z ew ex ey ez]
 """
 pos_err = x[0:3]-xd[0:3]
 qact = Quaternion(x[3:7])
 qdes = Quaternion(xd[3:7])
 qdif =  qdes*qact.inverse
 qua_err = np.array([qdif.w,qdif.x,qdif.y,qdif.z])
 err_pose = np.hstack((pos_err,qua_err))
 return err_pose

def dpinv(J, damp = 0.01):
 JT = J.T
 JJ_T = J @ JT
 I = np.eye(JJ_T.shape[0])
 J_dpinv = JT @ np.linalg.inv(JJ_T + (damp**2) * I)
 return J_dpinv

def penalty(q, qmin, qmax, w=1e-3):
    viol = np.maximum(0, q - qmax) + np.maximum(0, qmin - q)
    pen = w * np.sum(viol**2)  # penalidad cuadrada
    return pen

def ikine(xdes, q0, epsilon=0.001, max_iter=1000, delta=0.1):
 # límites articulares
 qmin = np.array([-3.12, -1.73, -1.73, 0, -3.12, -1.81, -3.12])
 qmax = np.array([3.12, 1.73, 1.73, 0.1, 3.12, 1.81, 3.12])
 q  = copy(q0)
 for i in range(max_iter):
  # Posición actual
  xact = fkine(q)
  xact = xact[0:3, 3]

  # Jacobiano
  J = jacobian(q, delta)
  # Error
  e = xdes - xact
  # calcular penalidad
  pen = penalty(q, qmin, qmax)
  # Error total
  e += pen
  # q nuevo
  q = q + np.linalg.pinv(J) @ e
  q = np.clip(q, qmin, qmax)

  # Break
  if (np.linalg.norm(e) < epsilon):
    break

  pass
    
 return q