%make human's path
t = 0:0.5:50;
x = t/3 + 6;
%Matt debugging - human go in straight line on x axis
y = 0.0*cos(3*x)+6;
% y = .5*sin(3*x)+4;
z = zeros(1,size(t,2));
%Matt Debugging 
% slope = .5*-3*sin(3*x);
slope = .5*3*cos(3*x);
psi = atan(slope);

%test plot to make sure direction and positioning are correct
plot(x,y,'r')
hold on
quiver(x,y,cos(psi),sin(psi))
axis equal

%save as timeseries into human structure
human.x = timeseries(x,t);
human.y = timeseries(y,t);
human.z = timeseries(z,t);
human.psi = timeseries(psi,t);

% save as .mat for later ;)
save('humanPathStraight.mat','human')
