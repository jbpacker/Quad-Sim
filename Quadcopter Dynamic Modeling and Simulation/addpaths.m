% a clean house is a happy house
clear all; close all; clc;

% add the path of the simulation pre-reqs
addpath(genpath('./Simulation Files/'));
addpath(genpath('./Graphical User Interfaces'));
addpath(genpath('./STLRead'));
addpath(genpath('./MattStuff'));

% Added needed library for lidar - not sure why entire path is not 
% being added?
addpath(genpath('./Lidar'));

% or load a map from file
load('./Lidar/ObstacleMap/map.mat');

% load current human path command
% load('./Simulation Files/Human Path Command/humanPath.mat');
% load('./humanPath.mat');
load('./Simulation Files/Human Path Command/humanPathStraight.mat');

% load camera noise files
load('./Simulation Files/Sensor Noise/noise.mat');

% load ICs
load('./MattStuff/ICMatt.mat');

% load QC path command (won't be needed soon)
load('./MattStuff/ChiCircle.mat');
% load('./MattStuff/ChiFollowing.mat');


% load QC structure
load('./Simulation Files/Quadcopter Structure Files/quadModel_+.mat');

% open my current simulink model
% open_system('./Simulation Files/Block Library/kalmanTest.slx');
% open_system('./Simulation Files/Simulink Models/PC_Quadcopter_SimulationPathPlanner.slx');
open_system('./Simulation Files/Simulink Models/AutoFollow_Simulation.slx');
