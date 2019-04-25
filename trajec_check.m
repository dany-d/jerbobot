% script to evaluate 
% trajectory for jerbobot

a_max = 50; %specified max acceleration, slope of trapezoid sides 
a_z = 30; 

% pt1
t_1 = 0;  % start time
x_1 = 0; % start x (global)
y_1 = 0; % start y (global)
z_1 = 0;
%pt2, next step
t_2 = 5;  
x_2 = 5;  
y_2 = 5;
z_2 = 0;

% convert global to omniwheel coordinates
xr_1 = x_1 * cos(pi/4) + y_1 * sin(pi/4);
yr_1 = -x_1 * sin(pi/4) + y_1 * cos(pi/4);
xr_2 = x_2 * cos(pi/4) + y_2 * sin(pi/4);
yr_2 = -x_2 * sin(pi/4) + y_2 * cos(pi/4);

% resulting changes in trajectory
dT = t_2-t_1;
dXr = abs(xr_2 - xr_1); % only care about magnitude of change
dYr = abs(yr_2 - yr_1);
dZ = abs(z_2 - z_1);

% poly for accel time (span of 
% symmetrical trapezoid sides)
% IF COMPLEX ROOTS, IMPOSSIBLE ROUTE FOR GIVEN ACCEL
p_xr = [1 -dT dXr/a_max]; % polynomial for xr
t_aXr = roots(p_xr) % print, take smallest value
p_yr = [1 -dT dYr/a_max];
t_aYr = roots(p_yr) 
p_z = [1 -dT dZ/a_z];
t_az = roots(p_z)
% if complex roots, impossible route


