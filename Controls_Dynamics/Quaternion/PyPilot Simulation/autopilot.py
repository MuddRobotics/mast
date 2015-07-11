__author__ = 'Kunal Menda, Peter Orme'

from math import *
import transformations as tf
import numpy as np
import random


class PIcontroller:
    def __init__(self,kp,ki,ulimit,llimit,Ts):
        self.kp = kp
        self.ki = ki
        self.ulimit = ulimit
        self.llimit = llimit
        self.Ts = Ts
        self.integrator = 0
        self.error_d1 = 0

    def sat(self,inp,ulimit,llimit):
        if inp > ulimit: return ulimit
        elif inp < llimit: return llimit
        else: return inp

    def update(self, var_c, var):
        error = var_c - var
        self.integrator += (self.Ts/2)*(error + self.error_d1)
        self.error_d1 = error
        u = self.sat(self.kp*error+self.ki*self.integrator,self.ulimit,self.llimit)
        # Implement Integrator Anti-Windup
        if self.ki != 0:
            u_unsat = self.kp*error+self.ki*self.integrator
            self.integrator = self.integrator + self.Ts/self.ki * (u - u_unsat)
        return u


class PIDcontroller:
    def __init__(self,kp,ki,kd,ulimit,llimit,Ts):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ulimit = ulimit
        self.llimit = llimit
        self.Ts = Ts
        self.integrator = 0
        self.error_d1 = 0

    def sat(self, inp, ulimit, llimit):
        if inp > ulimit: return ulimit
        elif inp < llimit: return llimit
        else: return inp

    def update(self, var_c, var, rate):
        error = var_c - var
        self.integrator += (self.Ts/2)*(error + self.error_d1)
        self.error_d1 = error
        u = self.sat(self.kp*error + self.ki*self.integrator - self.kd*rate, self.ulimit, self.llimit)
        # Implement Integrator Anti-Windup
        if self.ki != 0:
            u_unsat = self.kp*error+self.ki*self.integrator - self.kd*rate
            self.integrator = self.integrator + self.Ts/self.ki * (u - u_unsat)
        return u


class ErrorIntegrator:
    """ integrator with Anti-Windup"""
    def __init__(self, start_val, time):
        self.value = start_val # the current value of the integrator
        self.prev_error = start_val # the previous error measurement
        self.prev_time = time # the last time we have a measurement for

    def update(self, error, time):
        Ts = time - self.prev_time
        self.prev_time = time
        self.value += Ts*1.0/2*(error+self.prev_error) # trapezoidal integration
        return self.value

class MASTautopilot:
    def __init__(self):
        self.vel_int = None 
        self.z_int = None

    def update(self,uu):
        uu = map(float, uu)
        uu = np.array(uu, ndmin=2).transpose()
        # process inputs
        q_a      = uu[:4]        # Measured quaternion attitude
        w_B_a    = uu[4:7]       # Measured angular rates in body frame
        v_I_a    = uu[7:10]      # Measured velocity in inertial frame
        p_a      = uu[10:13]     # Measured position in intertial frame
        v_B_a    = uu[13:16]     # Measured velocity in body frame
        q_ref_d  = uu[16:20]     # Commanded reference quaternion
        v_I_d    = uu[20:23]     # Commanded velocity in inertial frame
        z_I_d    = uu[23]        # Commanded altitude
        t        = uu[-1]        # time

        # Assign constants
        m = 2 # kg
        g = 9.81 # m/s^2


        # PI velocity control
        # Gains (wtf are the values tho)
        K_p_yvel = 0.2
        K_i_yvel = 0.2

        K_p_xvel = 0.2
        K_i_xvel = 0.2

        K_p_z = 0.2
        K_v_z = 0.2

        v_I_e = v_I_d - v_I_a
        if self.vel_int is None:
            self.vel_int = ErrorIntegrator(v_I_e,t)
        else:
            self.vel_int.update(v_I_e,t)

        int_v_I_e = self.vel_int.value

        z_I_e = z_I_d - p_a[2]
        if self.z_int is None:
            self.z_int = ErrorIntegrator(z_I_e,t)
        else:
            self.z_int.update(z_I_e,t)
        int_z_I_e = self.z_int.value

        q_xy = np.array([[1.], 
                K_p_yvel * v_I_e[1] + K_i_yvel * int_v_I_e[1] +
                 K_p_z * np.sign(v_I_a[1])*z_I_e + K_v_z * np.sign(v_I_a[1]) * v_I_d[2],
                K_p_xvel * v_I_e[0] + K_i_xvel * int_v_I_e[0] +
                 K_p_z * np.sign(v_I_a[0])*z_I_e + K_v_z * np.sign(v_I_a[0]) * v_I_d[2],
                [0.]])

        q_d = tf.quaternion_multiply(q_ref_d.flatten(), q_xy.flatten())
        q_d = np.array(q_d, ndmin=2).transpose()



        # PD attitude control
        # Gains
        K_p_d_a = 1.4
        K_d_d_a = 0.2

        K_p_d_e = 2.0
        K_d_d_e = 0.25

        K_p_d_r = 1.7
        K_d_d_r = 0.1

        q_e = tf.quaternion_multiply(q_d.flatten(), tf.quaternion_conjugate(q_a.flatten()))
        q_e = np.array(q_e, ndmin=2).transpose()
        gamma_rotation = 2*cos(q_e[0])
        axis_vector = 1/(sin(gamma_rotation/2)) * q_e[1:]
        euler_errors = gamma_rotation*axis_vector
        K_p_euler = np.array([[K_p_d_a, 0., 0.], [0., K_p_d_e, 0.], [0., 0., K_p_d_r]])
        K_d_euler = np.array([[K_d_d_a, 0., 0.], [0., K_d_d_e, 0.], [0., 0., K_d_d_r]])
        u_w = np.dot(K_p_euler, euler_errors) + np.dot(K_d_euler, w_B_a) 


        # PID Thrust control
        # Gains
        K_p_d_t = 0.8 
        K_i_d_t = 0.2
        K_d_d_t = 0.33

        q_e_nominal = tf.quaternion_multiply(np.array([1,0,0,0]), tf.quaternion_conjugate(q_a.flatten()))
        gamma_rotation_nominal = 2*cos(q_e_nominal[0])
        axis_vector_nominal = 1/(sin(gamma_rotation_nominal/2)) * q_e_nominal[1:]
        euler_errors_nominal = gamma_rotation_nominal*axis_vector_nominal       
        if euler_errors_nominal[1] > 30*pi/180:
            Delta_v_B_x = v_I_e[1] / sin(euler_errors_nominal[1])
            delta_t = K_p_d_t * z_I_e + K_i_d_t * int_z_I_e - K_d_d_t * v_I_e[2] + K_v_x * Delta_v_B_x
        else:
            delta_t = m*g/cos(euler_errors_nominal[1]) + K_p_d_t * z_I_e + K_i_d_t * int_z_I_e - K_d_d_t * v_I_e[2]

        return np.append(u_w, [delta_t]) 







def makeAutopilot():
    return MASTautopilot()











def main():
    CaptainAppi = MASTautopilot()
    for i in range(100):
        inps = [random.random() for i in range(24)]
        print CaptainAppi.update(inps)

        # TODO 
        # Implement saturation
        # Implement anti windup







if __name__ == '__main__':
    main()