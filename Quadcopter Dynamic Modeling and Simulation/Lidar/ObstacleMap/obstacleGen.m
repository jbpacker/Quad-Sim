% Create a map (only run once for setup and save as .mat file)
R = 100; C = 100;
Xmax = 25; Ymax = 25;
map = makemap(R);
save('/home/matthew/Documents/gitHub/Quad-Sim/Quadcopter Dynamic Modeling and Simulation/Lidar/ObstacleMap/map.mat','map')