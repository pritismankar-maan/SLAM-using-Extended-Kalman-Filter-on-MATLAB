function [pred_current_pose,pred_cv] = predict_pose(current_pose,cv,...
                                                    count_landmarks,...
                                                    odometry,Fx)
% calculate odometry matrix to predict new pose. Since, the input from 
% odometry/controls can only manipulate the robot(not landmarks) pose ---
% current_pose(3): robot's previous angular value (theta)
odometry_matrix = [odometry.t*(cos(normalize_angle(current_pose(3)+odometry.r1)));
                   odometry.t*(sin(normalize_angle(current_pose(3)+odometry.r1)));
                          normalize_angle(odometry.r1+odometry.r2)            ];
% predicted current pose
new_pose = current_pose + transpose(Fx)*odometry_matrix; 

% calculate the new variance
% Jacobian matrix (3x3)
Gt_x = [1 0 -odometry.t*sin(current_pose(3)+odometry.r1);
        0 1  odometry.t*cos(current_pose(3)+odometry.r1);
        0 0  1];

% Jacobian matrix 
Gt = [Gt_x zeros(3,(2*count_landmarks));
      zeros((2*count_landmarks),3) eye(2*count_landmarks)];

% Odometry noise
Rt = [0.25 0     0;
       0   0.25  0;
       0   0    0.025];
R = transpose(Fx)*Rt*Fx;   

% predicted new covariance matrix
new_cv = [Gt_x*cv(1:3,1:3)*transpose(Gt_x) Gt_x*cv(1:3,4:end);
          transpose(Gt_x*cv(1:3,4:end))    cv(4:end,4:end)   ] + R;  

pred_cv = new_cv;
pred_current_pose = new_pose;

end