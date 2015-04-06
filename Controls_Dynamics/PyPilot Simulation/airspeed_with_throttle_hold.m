function u = airspeed_with_throttle_hold(V_c, V, flag,P)
Ts = P.Ts;
% Tau = P.Tau;
ulimit = 1;
llimit = 0;
ki = P.k_i_V;
kp = P.k_p_V;

persistent integrator;
 persistent error_d1;
 if flag==1, % reset (initialize) persistent variables
     % when flag==1
     integrator = 0;
     error_d1 = 0; % _d1 means delayed by one time step
 end
 error = V_c - V; % compute the current error
 integrator = integrator + (Ts/2)*(error + error_d1);
 % update integrator
 error_d1 = error; % update the error for next time through
 % the loop
 u = sat(... % implement PID control
 kp * error +... % proportional term
 ki * integrator,... % integral term
 ulimit,llimit... % ensure abs(u)<=limit
 );
 % implement integrator anti-windup
 if ki~=0
     u_unsat = kp*error + ki*integrator;
     integrator = integrator + Ts/ki * (u - u_unsat);
 end
 
    function out = sat(in, ulimit,llimit)
         if in > ulimit, out = ulimit;
         elseif in < -llimit; out = -llimit;
         else out = in;
         end
    end
end