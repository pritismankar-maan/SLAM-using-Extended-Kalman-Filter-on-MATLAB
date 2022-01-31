function h = drawellipse(x,a,b,color);

% Constants
NPOINTS = 100;                   % point density or resolution

% Compose point vector
ivec = 0:2*pi/NPOINTS:2*pi;     % index vector
p(1,:) = a*cos(ivec);           % 2 x n matrix which
p(2,:) = b*sin(ivec);           % hold ellipse points

% Translate and rotate
xo = x(1); yo = x(2); angle = x(3);
R  = [cos(angle) -sin(angle); sin(angle) cos(angle)];
T  = [xo; yo]*ones(1,length(ivec));
p = R*p + T;

% Plot
h = plot(p(1,:),p(2,:),'Color',color, 'linewidth', 2);
end