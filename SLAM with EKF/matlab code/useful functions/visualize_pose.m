function visualize_pose(current_pose,cv,landmarks,is_landmark_obs,sensor,...
                        start)
% In this function we are plotting the below things at each timestep:
%
% 1) Robot's estimated position by EKF Algorithm
% 2) Gaussian distribution(ellipse) around the Robot corrected position
% 3) Landmark's actual position
% 4) Landmark's estimated position by EKF Algorithm
% 5) Gaussian distribution(ellipse) around the Landmarks corrected position
% 6) Line joining the Robot and observed Landmark's estimated position. 

    clf;
    hold on
    grid("on")
    L = struct2cell(landmarks);
% plot Gaussian distribution(ellipse) around the Robot corrected position     
    drawprobellipse(current_pose(1:3), cv(1:3,1:3), 0.6, 'r');
% plot Landmark's actual position    
    plot(cell2mat(L(2,:)), cell2mat(L(3,:)), 'k+', 'markersize',...
                                              10, 'linewidth', 5);     
    for(i=1:length(is_landmark_obs))
        if(strcmp(is_landmark_obs(i).is_obs,'true'))
% for each observed landmark...
% plot Landmark's estimated position by EKF Algorithm            
            plot(current_pose(2*i+ 2),current_pose(2*i+ 3), ...
                 'bo', 'markersize', 10, 'linewidth', 5)
% plot Gaussian distribution(ellipse) around the Landmarks corrected position            
            drawprobellipse(current_pose(2*i+ 2:2*i+ 3),...
                            cv(2*i+ 2:2*i+ 3,2*i+ 2:2*i+ 3), 0.6, 'b');
        end
    end
% plot Line joining the Robot and observed Landmark's estimated position.
    for(i=1:size(sensor,2))
        mX = current_pose(2*sensor(i).id+2);
        mY = current_pose(2*sensor(i).id+3);
        line([current_pose(1), mX],[current_pose(2), mY], 'color', 'k', 'linewidth', 1);
    end
% plot Robot's estimated position by EKF Algorithm
    drawrobot(current_pose(1:3), 'r', 3, 0.3, 0.3);
    xlim([-2, 12])
    ylim([-2, 12])
    hold off
    drawnow;
    
% Capture the plot as an image 
    frame = getframe; 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    filename = 'SLAM with EKF.gif';
    if start == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append'); 
    end    
end