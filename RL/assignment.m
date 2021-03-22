% function assignment
% 
%     par = robot_set_parameters;
%     par.run_type = 'learn';
%     par = swingup(par);
%     
%     par.run_type = 'test';
%     [par, ta, xa] = swingup(par);
%     
% 
%     animate_swingup(ta, xa, par)
% end
clear all;
seed = 5;
rng(seed);


par = robot_set_parameters;
par.run_type = 'learn';
par = swingup(par);

par.run_type = 'test';
[par, ta, xa] = swingup(par);


animate_swingup(ta, xa, par)