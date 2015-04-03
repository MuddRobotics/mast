function u = pitch_hold(theta_c, theta, q, flag, P)

Ts = P.Ts;
tau = 1/P.w_n_theta;
limit = pi/4;
% ki = P.k_i_theta;
kp = P.k_p_theta;
kd = P.k_d_theta;

% persistent integrator;
 persistent differentiator;
 persistent error_d1;
 if flag==1, % reset (initialize) persistent variables
     % when flag==1
%      integrator = 0;
     differentiator = 0;
     error_d1 = 0; % _d1 means delayed by one time step
 end
 error = theta_c - theta; % compute the current error
%  integrator = integrator + (Ts/2)*(error + error_d1);
 % update integrator
 differentiator = (2*tau-Ts)/(2*tau+Ts)*differentiator...
 + 2/(2*tau+Ts)*(error - error_d1);
 % update differentiator
 error_d1 = error; % update the error for next time through
 % the loop
 u = sat(... % implement PID control
 kp * error +... % proportional term %  ki * integrator +... % integral term
 kd * q,... %  kd * differentiator,... % derivative term
 limit... % ensure abs(u)<=limit
 );

 
    function out = sat(in, limit)
         if in > limit, out = limit;
         elseif in < -limit; out = -limit;
         else out = in;
         end
    end
end