function [ out ] = PathPlannerFunBlock( x,y,z,psi, t )

% States [x, y, z, psi,time] of robot
% Camera data = [R_c, psi_c]

omega    = 2*pi/30; % rad/s of course angle chi
DT       = 1;       % s, step size in path planner
R        = 10;      % m, desired radius of path
                    % R*omega = 10*2*pi/30 = 2.09 m/s
% Fake camera data for one step (used for all steps)
R_c = 10;
psi_c = (pi/2);

% Current chi angle of path
chi_cur = states.psi + psi_c - pi/2;
chi_des = chi_cur - omega*DT;
psi_des = chi_des;

% Next Desired point
x_des = R*sin(chi_des);
y_des = R*cos(chi_des);

% Current x,y position of quad wrt human
x_cur = R_c*sin(chi_cur);
y_cur = R_c*cos(chi_cur);

% Attractivie force for VFF
d_t = sqrt( (x_des - x_cur)^2 + (y_des - y_cur)^2 );
C_A = 1;
Fx = C_A(x_des - x_cur)/d_t;
Fy = C_A(y_des - y_cur)/d_t;


x = timeseris(Fx,states.t+DT);
y = timeseries(Fy,states.t+DT);
z = timeseris(states.z,states.t+DT);
psi = timeseries(psi_des,states.t+DT);

out = [x;y;z;psi]
end

