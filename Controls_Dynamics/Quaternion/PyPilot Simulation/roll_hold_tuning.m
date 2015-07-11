clear
close all
clc

param_chap6

% Input Params

%%

delta_a_max = 45*pi/180;
phi_max = 15*pi/180;
e_phi_max = phi_max;
Va = 10; % m/s

a_phi_1 = -(1/2)*P.rho * Va^2 * P.S_wing * P.b * P.C_p_p * P.b / (2*Va);
a_phi_2 = (1/2)*P.rho * Va^2 * P.S_wing * P.b * P.C_p_delta_a;

% Tuning Params

zeta_phi = 0.55; % Tuning
k_i_phi = 1.05; %(small)


k_p_phi = delta_a_max/e_phi_max * sign(a_phi_2);

w_n_phi = sqrt(abs(a_phi_2) * delta_a_max/e_phi_max);

k_d_phi = (2*zeta_phi*w_n_phi - a_phi_1)/a_phi_2;

simm = sim('roll_loop.slx');

plot(simout.Time, simout.Data);

