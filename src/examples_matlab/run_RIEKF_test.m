%% Script to run simulation

% Reset 
clear; restoredefaultpath; clc;

% Add paths
addpath(genpath('forward_kinematics'))
addpath('InEKF')

%% Open Model  
mdl = 'RIEKF_test';
open_system(mdl); % Simulation model

%% Sim Model
sim(mdl);

%% Plot Results
plot_results;

