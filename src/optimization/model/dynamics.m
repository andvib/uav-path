%%  dynamics.m
%   Function called from Simulink that positional and angular derivatives
%   for integration in Simulink. Based on Chp 2-3 Fossen(2011), chp 2 Fossen(2014)
%   
%   Following SNAME naming convention
%   input:
%   output: F,m,
%   Verified against B&M eq 3.14-3.17
%   krisgry 10.09.14
%%
function output = dynamics(pos,quat,vel,Omega,tau)
    global P;
    persistent M_rb gamma first
    if(isempty(first))
        first = 0;
        gamma = 0; %convergence rate for quaternion normalization
        M_rb = [eye(3)*P.mass,     -P.mass*Smtrx(P.r_cg);
                P.mass*Smtrx(P.r_cg),      P.I_cg       ];
    end
    C_rb = [zeros(3),                                             -P.mass*Smtrx(vel)-P.mass*Smtrx(Omega)*Smtrx(P.r_cg);
            -P.mass*Smtrx(vel)+P.mass*Smtrx(P.r_cg)*Smtrx(Omega), -Smtrx(P.I_cg*Omega)]; %Fossen 3.56

    ny_dot = M_rb\(tau-C_rb*[vel;Omega]);       %(3.20)-(3.21) p 49 in Fossen(2011)
    [~,J1,T] = quatern(quat);

    pos_dot = J1*vel;                                    %(2.69) p30 in Fossen(2011)
    q_dot   = T*Omega -1/2*(1-quat'*quat)*quat;          %(2.73) p31 in Fossen(2011)
    
    output = [pos_dot',q_dot',ny_dot']';
end