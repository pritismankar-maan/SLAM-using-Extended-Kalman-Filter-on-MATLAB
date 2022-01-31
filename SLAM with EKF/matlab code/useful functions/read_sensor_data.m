function sensor_data = read_sensor_data(filename)
% Odometry reading (Input that dictates the robot motion)
% -------------------------------------------------------
% Here we are following the odometry based motion model so the motion model
% has 3 input commands(controls) - 

% 1) r1 - Intial angular rotation of the robot
% 2) t  - Translation distance after angle rotation of the robot
% 3) r2 - Angular rotation of the robot after translation

% This corresponds to Odometry structure inside the return variable. For each
% timestep there can one Odometry reading based on the input commands 
% provided by different control planning algorithm or manually.
% 
% Sensor reading (using LIDAR)
% ----------------------------
% The sensor data can observe multiple landmarks at a given point of time and
% return the range bearing (distance,angle between landmark and robot pose)
% for each of those landmarks.

% 1) id - To identify which landmark it observed (object correlation)
% 2) range - Distance of the landmark from robot
% 3) bearing - Angle between the landmark and the robot pose

% This corresponds to Sensor structure inside the return variable. For each 
% timestep there can be multiple sensor readings based on the landmark it can 
% see.


% read input file    
    input = fopen(filename,'r');
% create structure    
    sensor_data = struct;
    sensor_data.timestep.sensor = struct;
    count_timestep = 0;
% create array's of structure    
    while(~feof(input))
% read line of input file        
        line = fgetl(input);
% split the line into array based on spaces on the line        
        arr = strsplit(line, ' ');
% type of sensor data        
        type = deblank(arr{1});
        if(strcmp(type, 'ODOMETRY') == 1)
% If type of sensor data is Odometry            
            count_timestep = count_timestep+1;
            count_landmarks = 0;
% fill odometry data
            sensor_data.timestep(count_timestep).odometry.r1 = str2double(arr{2});
            sensor_data.timestep(count_timestep).odometry.t  = str2double(arr{3});
            sensor_data.timestep(count_timestep).odometry.r2 = str2double(arr{4});

        elseif(strcmp(type, 'SENSOR') == 1)
% If type of sensor data is Odometry
% for multiple landmarks observed at a given timestep
            count_landmarks = count_landmarks + 1;
% fill sensor data
            sensor_data.timestep(count_timestep).sensor(count_landmarks).id = str2double(arr{2});
            sensor_data.timestep(count_timestep).sensor(count_landmarks).range = str2double(arr{3});
            sensor_data.timestep(count_timestep).sensor(count_landmarks).bearing = str2double(arr{4});
        end
    end
% close file
    fclose(input);
    
end