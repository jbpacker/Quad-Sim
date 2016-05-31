% Create a map (only run once for setup and save as .mat file)
addpath(genpath('./../assignment6'));

Xmax = 25; Ymax = 25;
% obstacles
% x = [8, 18, 18]';
% y = [16, 12, 16]';
% clear map
x = [];
y = [];
obsLoc = [x,y];

obsRad = 1;

R = 100; C = 100;
map = zeros(R,C);

for i = 1:R
    for j = 1:C
        for o = 1:size(obsLoc,1)
            %distance between cell i,j and obs o
            [ x, y ] = IJtoXY(i, j, Xmax, Ymax, R, C);
            if (euclidianDist(x,y,obsLoc(o,1),obsLoc(o,2)) < obsRad)
                map(i,j) = 1;
                break;
            end
        end
    end
end

save('clearmap.mat','map')
save('clearGroundTruth.mat', 'obsLoc', 'obsRad')
% 
% save('obstaclemap.mat','map')
% save('obstacleGroundTruth.mat', 'obsLoc', 'obsRad')
