%make human's path
t = 0:0.5:50;
offset = 4;
x = t/3 + offset;
%Matt debugging - human go in straight line on x axis
% y = 0.0*cos(3*x)+6;
T = 18;
omega = 2*pi/T;
A = 2;
y = A*sin(omega*(x-offset))+12;
z = zeros(1,size(t,2));

slope = A*omega*cos(omega*(x-offset));
% slope = .5*3*cos(3*x);
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
save('humanPath.mat','human')
