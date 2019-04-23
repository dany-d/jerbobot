% script to evaluate 
% trajectory for jerbobot

a_max = 30; %specified max acceleration, slope of trapezoid sides 

t_1 = 0;  % start time
x_1 = 0; % start position
t_2 = 3; % next step time
x_2 = 13; % end position 

DT = t_2-t_1;
DX = abs(x_2-x_1); 

% polynominal for acceleration time (time duration of 
% symmetrical trapezoid sides)
t_p = [1 -DT DX/a_max];
t_a = roots(t_p) % output, take smallest value
% if complex roots, impossible route


