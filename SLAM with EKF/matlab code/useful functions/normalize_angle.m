function output = normalize_angle(input)
% the normalized angle should lie in between [-pi,pi]    
    while(input>pi)
        input = input-2*pi;
    end
    while(input<-pi)    
        input = input+2*pi;
    end
    output=input;
end