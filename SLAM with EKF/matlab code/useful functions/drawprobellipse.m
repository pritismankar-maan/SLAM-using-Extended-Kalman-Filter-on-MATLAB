function h = drawprobellipse(x,C,alpha,color);

% Calculate unscaled half axes
sxx = C(1,1); syy = C(2,2); sxy = C(1,2);
a = sqrt(0.5*(sxx+syy+sqrt((sxx-syy)^2+4*sxy^2)));   % always greater
b = sqrt(0.5*(sxx+syy-sqrt((sxx-syy)^2+4*sxy^2)));   % always smaller

% Remove imaginary parts in case of neg. definite C
if ~isreal(a), a = real(a); end;
if ~isreal(b), b = real(b); end;

% Scaling in order to reflect specified probability
a = a*sqrt(chi2invtable(alpha,2));
b = b*sqrt(chi2invtable(alpha,2));

% Look where the greater half axis belongs to
if sxx < syy, swap = a; a = b; b = swap; end;

% Calculate inclination (numerically stable)
if sxx ~= syy,
  angle = 0.5*atan(2*sxy/(sxx-syy));	
elseif sxy == 0,
  angle = 0;     % angle doesn't matter 
elseif sxy > 0,
  angle =  pi/4;
elseif sxy < 0,
  angle = -pi/4;
end;
x(3) = angle;

% Draw ellipse
h = drawellipse(x,a,b,color);
end