% plot human path, quadcopter path, and obstacles
close all
% Human path ( col 27 and 28 of yout)
figure(1)
plot(yout(:,27),yout(:,28))
hold on
% QuadPath ( Col 10 and 11 of yout)
plot(yout(:,10)*3.28,yout(:,11)*3.28,'g');

xlabel('x (ft)')
ylabel('y (ft)')

% Obstacle map
% compute obstacles from map in world frame
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

% add obstacle to plot
scatter(xObs,yObs,'r')

legend('human','copter','obstacles')