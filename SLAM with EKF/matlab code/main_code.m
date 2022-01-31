%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%        SLAM using Extended Kalman Filter
%               Name - Pritisman Kar
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;close all;clc;

% read landmark as a structure from input text file 
[landmarks,is_landmark_obs] = read_lankmarks('C:\Users\HP\Documents\MATLAB\SLAM with EKF\sensor_data\world.txt');
% read sensor data as a structure from input text file 
sensor_data = read_sensor_data('C:\Users\HP\Documents\MATLAB\SLAM with EKF\sensor_data\sensor_data.txt');

% Intialization at t = 0
% ----------------------
% count the number of landmarks so as to set the size of be pose vector
% and covariance matrix. In reality, this vector and matrix should grow based
% on the new landmakrs observed by the sensor.
count_landmarks = size(landmarks,2);
% initial pose of the robot and the location of landmarks
current_pose = zeros((2*count_landmarks+3),1);
% initial covariance matrix setup
% % covariance matrix = [0 0 0  .   .  . . . . .  0;
%                        0 0 0  .   .  . . . . .  0;
%                        0 0 0 inf  .  . . . . .  0;
%                        0 0 0  .  inf . . . . .  0;
%                        . . .  .   .  . . . . .  0
%                        0 0 0  .   .  . . . . . inf;]    
%  matrix size: [(2*count_landmarks+3)*(2*count_landmarks+3)]

inf = 9999;
cv_of_robot_pose = zeros(3,3);
cv_of_map_robot = zeros(2*count_landmarks,3);
cv_of_robot_map = zeros(3,2*count_landmarks);
cv_of_map_map = inf*eye(2*count_landmarks);
cv = [cv_of_robot_pose cv_of_robot_map;
      cv_of_map_robot  cv_of_map_map];  

% calculate 'Fx' matrix (mapping matrix) to predict the updated robot's and 
% landmark poses based on input odometry command, time & environment. Here,
% the landmarks are assumed to be still and hence 'Fx_map_pose' matrix is a
% zero matrix.
Fx_robot_pose = eye(3);
Fx_map_pose = zeros(3,2*count_landmarks);
Fx = [Fx_robot_pose Fx_map_pose];  
% Run algorithm (t > 0)
% ----------------------   

for i=1:size(sensor_data.timestep,2)
%   perform prediction step to get predict pose and covariance matrix
    [pred_current_pose,pred_cv] = predict_pose(current_pose,cv,count_landmarks,...
                                     sensor_data.timestep(i).odometry,Fx);
%   perform correction step to get corrected pose and covariance matrix
%   based on observation data
    [corr_current_pose,corr_cv,corr_landmark_obs] = correct_pose(...
                                            pred_current_pose,pred_cv,...
                                            count_landmarks,...
                                            sensor_data.timestep(i).sensor,...
                                            is_landmark_obs);
%   update the corrected pose as the current pose to be used in the next
%   iteration
    current_pose = corr_current_pose;
    cv= corr_cv;
    is_landmark_obs = corr_landmark_obs;
%   display plot with each timestep    
    visualize_pose(current_pose,cv,landmarks,is_landmark_obs,...
                   sensor_data.timestep(i).sensor,i);
end