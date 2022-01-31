function [landmarks,is_landmark_obs] = read_lankmarks(filename)
%  Filename contains the actual landmark location in the (x,y) plane
%  The return variable contains 3 fields - 
%  1) id - To identify which landmark it observed (object correlation)
%  2) x  - The x value of the given landmark
%  3) y  - The y value of the given landmark

% open the input file
input = fopen(filename,'r');
% create a structure
landmarks = struct('id',[],'x',[],'y',[]); 
is_landmark_obs = struct('id',[],'is_obs',[]);

count = 0;
% create array of structure
    while(~feof(input))
        count = count + 1;
% read the current line of the text file        
        line = fgetl(input);
% split the line based on spaces present in it        
        data = strsplit(line, ' ');
% fill landmark information        
        landmarks(count).id = str2double(data{1});
        landmarks(count).x = str2double(data{2});
        landmarks(count).y = str2double(data{3});
% fill 'is landmark observed' structure
        is_landmark_obs(count).id = str2double(data{1});
        is_landmark_obs(count).is_obs = 'false';
    end
% close the input file
    fclose(input);
end