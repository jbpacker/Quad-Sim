%make human's path
t = 0:0.5:45;
xoffset = 4;
x = t/3 + xoffset;
%Matt debugging - human go in straight line on x axis
% y = 0.0*cos(3*x)+6;
T = 15;
y = 4*sin(2*pi/(T)*(x-xoffset))+12;
z = zeros(1,size(t,2));
%Matt Debugging 
% slope = .5*-3*sin(3*x);
slope = 5*2*pi/(T)*cos(2*pi/(T)*(x - xoffset));
psi = atan(slope);

%test plot to make sure direction and positioning are correct
% xobs = [8, 9, 14, 15];
% yobs = [14,19,20, 10];   
plot(x,y,'r')
hold on
quiver(x,y,cos(psi),sin(psi))
% scatter(xobs, yobs, 'g')
axis equal

%save as timeseries into human structure
human.x = timeseries(x,t);
human.y = timeseries(y,t);
human.z = timeseries(z,t);
human.psi = timeseries(psi,t);

% save as .mat for later ;)
save('humanPath.mat','human')
