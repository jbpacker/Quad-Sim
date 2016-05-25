xq = 3;
yq = 4;

% min and max of world coordinates (sees obtacle at edge)
Xmax = 25; Ymax = 25;

% span of scanner and step size
angleSpan = 2*pi; angleStep = 2*pi/360;
%(m) max range of lidar scanner 
rangeMax=50; 

%current laser pose (coordinate frame) wrt world
% Laser located in vehcile frame of quadcopter
Tl=se2([xq yq 0]); 

% p = laserScanner(angleSpan, angleStep, rangeMax, Tl, map, Xmin, Ymin, Xmax, Ymax)
% return (theta,r) of scan
p = laserScanner(angleSpan, angleStep, rangeMax, Tl, map, Xmax, Ymax); %p contains angle and range readings

figure(2);
plot(p(:,1)*180/pi, p(:,2)); % plot angle and range - it is displayed in ccw, from -angleSpan/2 to +angleSpan/2


r = p(:,2); theta = p(:,1);
% remove data points beyond threshold of rangeMax
i = 1;
while i < length(r)
if r(i) >= rangeMax
    r = [r(1:i-1); r(i+1:end)];
    theta = [theta(1:i-1); theta(i+1:end)];
    i = i-1;
end
i = i+1;
end

x = zeros(1,length(r));
y = zeros(1,length(r));
% convert r,theta to cartesian of obstacles found from robot perspective
for i = 1:length(r)
    x(i) = r(i)*cos(theta(i));
    y(i) = r(i)*sin(theta(i));
end

% compare results to actual world map frame obstacle
R = size(map,1); C = size(map,2);
xObs = 0;
yObs = 0;
for i = 1:R
    for j = 1:C
        if map(i,j) == 1
            [ xTarget, yTarget ] = IJtoXY(i, j, Xmax, Ymax, R, C);
            xObs = [xObs, xTarget];
            yObs = [yObs, yTarget];
        end
    end
end


figure(3)
scatter(x,y)
figure(4)
scatter(xObs,yObs)