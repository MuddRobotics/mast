% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % Rotation from body to vehicle frame
    
      R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
      R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
      R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
      R = R_roll*R_pitch*R_yaw;  
    
    % compute wind data in NED
    V_w = R*[w_ns; w_es; w_ds] + [u_wg; v_wg; w_wg;];
    u_r = u-V_w(1);
    v_r = v-V_w(2);
    w_r = w-V_w(3);
    
    V_w_v = R'*V_w;
    w_n = V_w_v(1);
    w_e = V_w_v(2);
    w_d = V_w_v(3);
    
    % compute air data
    Va = sqrt(u_r^2 + v_r^2 + w_r^2);
    alpha = atan2(w_r,u_r);
    beta = asin(v_r/Va);
    
    % compute external forces and torques on aircraft
    Force(1) =  (-P.m*P.gravity*sin(theta)) ...
                + .5*P.rho*Va^2*P.S_wing*(-(P.C_D_0+P.C_D_alpha*alpha)*cos(alpha)...
                + (P.C_L_0+P.C_L_alpha*alpha)*sin(alpha)...
                + (-P.C_D_q*cos(alpha)+P.C_L_q*sin(alpha))*P.c*q/(2*Va) ...
                + (-P.C_D_delta_e*cos(alpha)+P.C_L_delta_e*sin(alpha))*delta_e) ...
                + .5*P.rho*P.S_prop*P.C_prop*((P.k_motor*delta_t)^2-Va^2);
    Force(2) =  (P.m*P.gravity*cos(theta)*sin(phi)) ...
                + .5*P.rho*Va^2*P.S_wing*(P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*P.b*p/(2*Va)...
                + P.C_Y_r*P.b*r/(2*Va) + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r);
    Force(3) =  (P.m*P.gravity*cos(theta)*cos(phi)) ...
                + .5*P.rho*Va^2*P.S_wing*(-(P.C_D_0+P.C_D_alpha*alpha)*sin(alpha) ...
                - (P.C_L_0+P.C_L_alpha*alpha)*cos(alpha) ...
                + (-P.C_D_q*sin(alpha)-P.C_L_q*cos(alpha))*P.c*q/(2*Va) ...
                + (-P.C_D_delta_e*sin(alpha)-P.C_L_delta_e*cos(alpha))*delta_e);
    
    Torque(1) = P.b*(P.C_ell_0 + P.C_ell_beta*beta + P.C_ell_p*P.b*p/(2*Va)...
                + P.C_ell_r*P.b*r/(2*Va) + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r);
    Torque(2) = P.c*(P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*P.c*q/(2*Va) ...
                + P.C_m_delta_e*delta_e);   
    Torque(3) = P.b*(P.C_n_0 + P.C_n_beta*beta + P.C_n_p*P.b*p/(2*Va) ...
                + P.C_n_r*P.b*r/(2*Va) + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r);
   
    Torque = Torque*.5*P.rho*Va^2*P.S_wing;
    Torque(1) = Torque(1) - P.k_T_P*(P.k_Omega*delta_t)^2;
    
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



