%make desired chi angle for path
omega    = 2*pi/60; % rad/s of course angle chi (1 rev around human / min)
DT       = 1;       % s, step size in path planner
t = 0:DT:50;
% negative rotation = cw from -x axis
% chi = 0:-omega*DT:-max(t)*omega;
% Debuggin chi = 0 degrees, follow human behind
chi = zeros(1,size(t,2));

%test plot to make sure direction and positioning are correct
plot(t,chi,'r')
axis equal

%save as timeseries into human structure
Chi = timeseries(chi,t);

% save as .mat for later ;)
save('ChiFollowing.mat','Chi')