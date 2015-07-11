__author__ = 'Kunal Menda, Peter Orme'

from math import pi


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


class MASTautopilot:
    def __init__(self, Params):
        self.roll_hold = PIDcontroller(Params['k_p_phi'],Params['k_i_phi'],Params['k_d_phi'],pi/4,-pi/4,Params['Ts'])
        self.course_hold = PIcontroller(Params['k_p_chi'],Params['k_i_chi'],25*pi/180,-25*pi/180,Params['Ts'])
        self.coordinated_turn_hold = PIcontroller(-Params['k_p_beta'],-Params['k_i_beta'], \
                                                  35*pi/180,-35*pi/180,Params['Ts']) # Note the negation on the gains.
        self.pitch_hold = PIDcontroller(Params['k_p_theta'],0,Params['k_d_theta'],pi/4,-pi/4,Params['Ts'])
        self.altitude_hold = PIcontroller(Params['k_p_h'],Params['k_i_h'],10*pi/180,-10*pi/180,Params['Ts'])
        self.airspeed_with_pitch_hold = PIcontroller(Params['k_p_V2'],Params['k_i_V2'],10*pi/180,-10*pi/180,Params['Ts'])
        self.airspeed_with_throttle_hold = PIcontroller(Params['k_p_V'],Params['k_i_V'],1,0,Params['Ts'])

        self.altitude_take_off_zone = Params['altitude_take_off_zone']
        self.altitude_hold_zone = Params['altitude_hold_zone']
        self.theta_takeoff = Params['theta_takeoff']
        self.delta_t_trim = Params['delta_t_trim']


    def update(self,uu):

        uu = map(float, uu)

        # process inputs
        q_a      = uu[:4]    # Measured quaternion attitude
        w_B_a    = uu[4:7]   # Measured angular rates in body frame
        v_I_a    = uu[7:10]  # Measured velocity in inertial frame
        p_a      = uu[10:13] # Measured position in intertial frame
        v_B_a    = uu[13:16] # Measured velocity in body frame
        t        = uu[16]    # time

        # Lateral Autopilot
        phi_c   = self.course_hold.update(chi_c, chi)
        delta_r = self.coordinated_turn_hold.update(0, beta)
        delta_a = self.roll_hold.update(phi_c, phi, p)

        # Longitudinal Autopilot

        if h <= self.altitude_take_off_zone:
            altitude_state = 1
        elif h <= h_c-self.altitude_hold_zone:
            altitude_state = 2
        elif h >= h_c+self.altitude_hold_zone:
            altitude_state = 3
        else:
            altitude_state = 4

        delta_t_trim = self.delta_t_trim  # there should be a better way

        awp = self.airspeed_with_pitch_hold.update(Va_c, Va)
        awt = delta_t_trim + self.airspeed_with_throttle_hold.update(Va_c, Va)
        ah = self.altitude_hold.update(h_c, h)

        if altitude_state == 1:  # take off zone
            delta_t = 1
            theta_c = self.theta_takeoff
        elif altitude_state == 2:  # climb zone
            delta_t = 1
            theta_c = awp
        elif altitude_state == 3:  # descend zone
            delta_t = 0
            theta_c = awp
        elif altitude_state == 4:  # altitude hold zone
            delta_t = awt
            theta_c = ah

        delta_e = self.pitch_hold.update(theta_c, theta, q)

        # Return control inputs
        #return [float(delta_e), float(delta_a), float(delta_r), float(delta_t)]
        return [0,0,0,0]




def makeAutopilot():
    Params = dict()
    import csv
    with open('ParamsForPyAutopilot.csv', 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in reader:
            Params[row[0]] = float(row[1])
        return MASTautopilot(Params)











def main():
    import csv


    Params = dict()

    with open('ParamsForPyAutopilot.csv', 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in reader:
            Params[row[0]] = float(row[1])

    Inputs = []
    Outputs = []
    with open('PyTestInputs.csv', 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in reader:
            myrow = [float(r) for r in row]
            Inputs.append(myrow)
    with open('PyTestOutputs.csv', 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in reader:
            myrow = [float(r) for r in row]
            Outputs.append(myrow[1:5])


    MyAutopilot = MASTautopilot(Params)

    SimOutputs = []
    for row in Inputs[0:10]:
        print 'Inputs: ',row
        SimOutputs.append(MyAutopilot.update(row[1:]))

    for i in range(0, len(SimOutputs)):
        print [100*(a_i - b_i)/(b_i+0.00000001) for a_i, b_i in zip(SimOutputs[i], Outputs[i])]
        # print [a_i*180/pi for a_i in SimOutputs[i][:-1]].append(SimOutputs[i][-1])






if __name__ == '__main__':
    main()