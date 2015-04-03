% use these revised values for the Aerosonde UAV

P.mass = 25;
% initial airspeed
P.Va0 = 35;        % m/s (~85 mph)

% autopilot sample rate
P.Ts = 0.01;

P.gravity = 9.8;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%physical parameters of airframe
P.m = 13.5;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = .1204;
% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;


P.GAMMA  = (P.Jx * P.Jz) - (P.Jxz^2);
P.GAMMA1 = (P.Jxz * (P.Jx - P.Jy) + P.Jxz^2) / P.GAMMA; 
P.GAMMA2 = ((P.Jz * (P.Jz - P.Jy)) + P.Jxz^2) / P.GAMMA;
P.GAMMA3 = (1 / P.GAMMA) * P.Jz;
P.GAMMA4 = (1 / P.GAMMA) * P.Jxz;
P.GAMMA5 = (1 / P.Jy) * (P.Jz - P.Jx);
P.GAMMA6 = P.Jxz / P.Jy;
P.GAMMA7 = (1 / P.GAMMA) * ((P.Jx -P.Jy)*P.Jx + (P.Jxz)^2);
P.GAMMA8 = P.Jx / P.GAMMA;

P.C_p_0 = P.GAMMA3*P.C_ell_0 + P.GAMMA4*P.C_n_0;
P.C_p_beta = P.GAMMA3*P.C_ell_beta + P.GAMMA4*P.C_n_beta;
P.C_p_p = P.GAMMA3*P.C_ell_p+ P.GAMMA4*P.C_n_p;
P.C_p_r = P.GAMMA3*P.C_ell_r + P.GAMMA4*P.C_n_r;
P.C_p_delta_a = P.GAMMA3*P.C_ell_delta_a + P.GAMMA4*P.C_n_delta_a;
P.C_p_delta_r = P.GAMMA3*P.C_ell_delta_r + P.GAMMA4*P.C_n_delta_r;

P.C_r_0 = P.GAMMA4*P.C_ell_0 + P.GAMMA8*P.C_n_0;
P.C_r_beta = P.GAMMA4*P.C_ell_beta + P.GAMMA8*P.C_n_beta;
P.C_r_p = P.GAMMA4*P.C_ell_p+ P.GAMMA8*P.C_n_p;
P.C_r_r = P.GAMMA4*P.C_ell_r + P.GAMMA8*P.C_n_r;
P.C_r_delta_a = P.GAMMA4*P.C_ell_delta_a + P.GAMMA8*P.C_n_delta_a;
P.C_r_delta_r = P.GAMMA4*P.C_ell_delta_r + P.GAMMA8*P.C_n_delta_r;

P.C_L = @(alpha) P.C_L_0 + P.C_L_alpha*alpha;
P.C_D = @(alpha) P.C_D_0 + P.C_D_alpha*alpha;

P.C_X = @(alpha) -P.C_D(alpha)*cos(alpha) +P.C_L(alpha)*sin(alpha);
P.C_X_q = @(alpha) -P.C_D_q*cos(alpha) +P.C_L_q*sin(alpha);
P.C_X_delta_e = @(alpha) -P.C_D_delta_e*cos(alpha) +P.C_L_delta_e*sin(alpha);

P.C_Z = @(alpha) -P.C_D(alpha)*sin(alpha) - P.C_L(alpha)*cos(alpha);
P.C_Z_q = @(alpha) -P.C_D_q*sin(alpha) - P.C_L_q*cos(alpha);
P.C_Z_delta_e = @(alpha) -P.C_D_delta_e*sin(alpha) - P.C_L_delta_e*cos(alpha);


% wind parameters
P.wind_n = 0;%3;
P.wind_e = 0;%2;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = .7;


% compute trim conditions using 'mavsim_chap5_trim.slx'
% initial airspeed
P.Va0 = 35;
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = Inf;         % desired radius (m) - use (+) for right handed orbit, 

% autopilot sample rate
P.Ts = 0.01;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

                    %                          (-) for left handed orbit

% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
% P.pn0    = 0;  % initial North position
% P.pe0    = 0;  % initial East position
% P.pd0    = 0;  % initial Down position (negative altitude)
% P.u0     = x_trim(4);  % initial velocity along body x-axis
% P.v0     = x_trim(5);  % initial velocity along body y-axis
% P.w0     = x_trim(6);  % initial velocity along body z-axis
% P.phi0   = x_trim(7);  % initial roll angle
% P.theta0 = x_trim(8);  % initial pitch angle
% P.psi0   = x_trim(9);  % initial yaw angle
% P.p0     = x_trim(10);  % initial body frame roll rate
% P.q0     = x_trim(11);  % initial body frame pitch rate
% P.r0     = x_trim(12);  % initial body frame yaw rate

P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = 10;  % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate



% % compute different transfer functions
% [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
%     = compute_tf_model(x_trim,u_trim,P);
% 
% % linearize the equations of motion around trim conditions
% [A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

tuning;

P.altitude_take_off_zone = 30;
P.altitude_hold_zone = 10;
P.theta_takeoff = 10*pi/180;

P.k_d_phi = k_d_phi;
P.k_d_theta = k_d_theta;
P.k_i_beta = k_i_beta;
P.k_i_chi = k_i_chi;
P.k_i_V = k_i_V;
P.k_i_V2 = k_i_V2;
P.k_i_phi = k_i_phi;
P.k_i_h = k_i_h;
% P.k_i_theta = k_i_theta;
P.k_p_beta = k_p_beta;
P.k_p_chi = k_p_chi;
P.k_p_h = k_p_h;
P.k_p_phi = k_p_phi;
P.k_p_theta = k_p_theta;
P.k_p_V = k_p_V;
P.k_p_V2 = k_p_V2;
P.w_n_phi = w_n_phi;
P.w_n_theta = w_n_theta;

