function u = roll_hold(phi_c, phi, p, flag, P)

Ts = P.Ts;
tau = 1/P.w_n_phi;
limit = pi/4;
ki = P.k_i_phi;
kp = P.k_p_phi;
kd = P.k_d_phi;

persistent integrator;
 persistent differentiator;
 persistent error_d1;
 if flag==1, % reset (initialize) persistent variables
     % when flag==1
     integrator = 0;
     differentiator = 0;
     error_d1 = 0; % _d1 means delayed by one time step
 end
 error = phi_c - phi; % compute the current error
 integrator = integrator + (Ts/2)*(error + error_d1);
 % update integrator
 differentiator = (2*tau-Ts)/(2*tau+Ts)*differentiator...
 + 2/(2*tau+Ts)*(error - error_d1);
 % update differentiator
 error_d1 = error; % update the error for next time through
 % the loop
 u = sat(... % implement PID control
 kp * error +... % proportional term
 ki * integrator +... % integral term
 kd * -p,... %  kd * differentiator,... % derivative term
 limit... % ensure abs(u)<=limit
 );
 % implement integrator anti-windup
 if ki~=0
     u_unsat = kp*error + ki*integrator + kd*differentiator;
     integrator = integrator + Ts/ki * (u - u_unsat);
 end
 
    function out = sat(in, limit)
         if in > limit, out = limit;
         elseif in < -limit; out = -limit;
         else out = in;
         end
    end
end