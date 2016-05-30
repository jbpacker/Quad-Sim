% Create a map (only run once for setup and save as .mat file)
addpath(genpath('./../assignment6'));

Xmax = 25; Ymax = 25;
numObs = 6;
% x = Xmax.*rand(numObs,1);
% y = Ymax.*rand(numObs,1);
xobs = [8, 9, 14, 16]';
yobs = [12,19,20, 10]';
obsLoc = [xobs,yobs];
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

save ./map.mat map
save ./GroundTruth.mat obsLoc obsRad


