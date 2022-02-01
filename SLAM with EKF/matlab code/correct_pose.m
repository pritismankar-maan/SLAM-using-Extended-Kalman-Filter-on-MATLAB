function [corr_current_pose,corr_cv,corr_landmark_obs] = correct_pose(...
                                                    pred_current_pose,...
                                                    pred_cv,count_landmarks,...
                                                    sensor,is_landmark_obs)
%   introduce sensor noise (variance = sqrt(0.05))
    Q = eye(2)*0.25;
    
% calculate the observed position of landmarks
for i=1:size(sensor,2)
%   for each observed landmark, calculate it's location based on range
%   bearing sensor formula. Calculate the observed landmark data and
%   predicted landmark data and correct the predicted pose and covariance. 
%   data     = [range ; bearing]
%   location = [x ; y]
%   Note: If a landmark is observed for the first time, the observed data
%   and the predicted data should be the same.
    obs_data = [sensor(i).range;sensor(i).bearing]; 
    obs_loc = [pred_current_pose(1) + ...
               sensor(i).range*cos(normalize_angle(sensor(i).bearing+...
                                                pred_current_pose(3)));
               pred_current_pose(2) + ...
               sensor(i).range*sin(normalize_angle(sensor(i).bearing+...
                                                pred_current_pose(3)))];

% if the landmark is to be seen for the first time, initilize the
% landmark's position based on the observed location. Only when we have the
% same landmark's location from another timestep, then only we can predict
% and correct to the landmark's actual location on the map.
    lis_landmark_obs = is_landmark_obs([sensor(i).id]);
    if(strcmp(lis_landmark_obs.is_obs,'false'))
        pred_current_pose(2+2*sensor(i).id) = obs_loc(1);
        pred_current_pose(3+2*sensor(i).id) = obs_loc(2);
        is_landmark_obs([sensor(i).id]).is_obs = 'true';
           
    end
% calculate error in the location of landmarks   
    err_loc = [obs_loc(1) - pred_current_pose(1);
               obs_loc(2) - pred_current_pose(2)];

% calculate certain values, to evalute Jacobian & predicted landmark's location    
    q = transpose(err_loc)*err_loc;
    pred_data    = [                    sqrt(q(1)); 
                       normalize_angle(atan2(err_loc(2),err_loc(1)) ...
                   - pred_current_pose(3))];
    
    if(strcmp(lis_landmark_obs.is_obs,'false'))
%   Assume Kt and Ht value as matlab can't handle NaN or Infinity value
    Kt=zeros(21,2);Ht=zeros(2,21);
    else    
%   compute the low jacobian
    Ht_low = 1/q(1)*[-sqrt(q(1))*err_loc(1) -sqrt(q(1))*err_loc(2)  0   ...
                      sqrt(q(1))*err_loc(1) -sqrt(q(1))*err_loc(1);
                           err_loc(2)          -err_loc(1)        -q(1) ...
                          -err_loc(2)           err_loc(1)];
            
%   create the mapping function Fx to compute Jacobian
    Fx = [eye(3)     zeros(3,2*sensor(i).id-2) zeros(3,2) zeros(3,2*count_landmarks-2*sensor(i).id);
          zeros(2,3) zeros(2,2*sensor(i).id-2)   eye(2)   zeros(2,2*count_landmarks-2*sensor(i).id)];

%   compute Jacobian    
    Ht = Ht_low*Fx;
%     Ht = [H;Hi];
%   compute Kalman gain
    Kt = pred_cv*transpose(Ht)*inv(Ht*pred_cv*transpose(Ht) + Q);
    end
%   correct robot pose,landmarks pose & covariance matrix with every iteration    
    pred_current_pose = pred_current_pose + Kt*normalize_diff(obs_data - ...
                                                              pred_data);
    pred_cv = (eye(2*count_landmarks+3) - Kt*Ht)*pred_cv;
end
%   update the return variables
    corr_current_pose = pred_current_pose;
    corr_cv = pred_cv;
    corr_landmark_obs = is_landmark_obs;
end