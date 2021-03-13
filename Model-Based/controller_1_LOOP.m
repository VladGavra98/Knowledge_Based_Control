clc
clearvars
close all
clear;


rp = define_robot_parameters();
sim_time = 10; % simualtion time in seconds
dt = 0.03; % time difference in seconds
t = 0:dt:sim_time;


%% INITIAL RUN TO MAKE A 3D MATRIX TO WHICH THEN WE CONTACTENATE 
%copied from controller_1
% DESIRED TRAJECTORY DATA
d2r  = pi/180;             % degrees to radians
tp.w = 72*d2r;            % rotational velocity rad/s
tp.rx = 1.75; tp.ry = 1.25; % ellipse radii
tp.ell_an = 45*d2r;       % angle of inclination of ellipse
tp.x0 = 0.4;  tp.y0 = 0.4;  % center of ellipse  
% Calculate desired trajectory in task space and in joint space
des = calculate_trajectory(t, tp, rp);
th_0 = des.th(:,1) - [0.1; 0.2];
th_d_0 = des.th_d(:,1);
% SIMULATE ROBOT
Kp = [500; 500];
Kd = [50; 50];
curr = simulate_robot(t, dt, th_0, th_d_0, des, rp, ...
    @(th_curr, th_d_curr, th_des, th_d_des, th_dd_des) ff_dyn_model_1(th_curr, th_d_curr, th_des, th_d_des, th_dd_des, rp), ...
    @(th_curr, th_d_curr, th_des, th_d_des) fb_pd(th_curr, th_d_curr, th_des, th_d_des, Kp, Kd));

%New: buildthe 1st 3d matrix
%put each current state in a vector, odd is J1, even J2
curr_x      = zeros(1,668);
curr_x_d    = zeros(1,668);
curr_x_dd   = zeros(1,668);
curr_x_eb   = zeros(1,668);
curr_th     = zeros(1,668);
curr_th_d   = zeros(1,668);
curr_th_dd  = zeros(1,668);
curr_tau_ff = zeros(1,668);
curr_tau_fb = zeros(1,668);
    
for i = 1:668
        %disp(curr.x(i));
        curr_x(i)      = curr.x(i); %vector of x's: odd is first joint, even is second
        curr_x_d(i)    = curr.x_d(i);
        curr_x_dd(i)   = curr.x_dd(i);
        curr_x_eb(i)   = curr.x_eb(i);
        curr_th(i)     = curr.th(i);
        curr_th_d(i)   = curr.th_d(i);
        curr_th_dd(i)  = curr.th_dd(i);
        curr_tau_ff(i) = curr.tau_ff(i);
        curr_tau_fb(i) = curr.tau_fb(i);
end
%separate the full vector between even and odd joints
% odd index values (1st joint)
curr_x1    = curr_x(1:2:end) ;
curr_x_d1  = curr_x_d(1:2:end) ;
curr_x_dd1 = curr_x_dd(1:2:end) ;
curr_x_eb1 = curr_x_eb(1:2:end);
curr_th1   = curr_th(1:2:end);
curr_th_d1 = curr_th_d(1:2:end) ;
curr_th_dd1 = curr_th_dd(1:2:end) ;
curr_tau_ff1 = curr_tau_ff(1:2:end) ;
curr_tau_fb1 = curr_tau_fb(1:2:end) ;
% even index values (2nd joint)
curr_x2    = curr_x(2:2:end) ;
curr_x_d2  = curr_x_d(2:2:end) ;
curr_x_dd2 = curr_x_dd(2:2:end) ;
curr_x_eb2 = curr_x_eb(2:2:end);
curr_th2  = curr_th(2:2:end);
curr_th_d2 = curr_th_d(2:2:end) ;
curr_th_dd2 = curr_th_dd(2:2:end) ;
curr_tau_ff2 = curr_tau_ff(2:2:end) ;
curr_tau_fb2 = curr_tau_fb(2:2:end) ;
    
%create a 2x334 matrix for each state (feature)
curr_xs    = [curr_x1;curr_x2];     %should yield the same as curr.x
curr_x_ds  = [curr_x_d1;curr_x_d2];
curr_x_dds = [curr_x_dd1;curr_x_dd2];
curr_x_ebs = [curr_x_eb1;curr_x_eb2];
curr_ths = [curr_th1;curr_th2];
curr_th_ds = [curr_th_d1;curr_th_d2];
curr_th_dds = [curr_th_dd1;curr_th_dd2];
curr_tau_ffs = [curr_tau_ff1;curr_tau_ff2];
curr_tau_fbs = [curr_tau_fb1;curr_tau_fb2];
  
%start the 1st 3D matrix
CURR_BIG = cat(3, curr_xs, curr_x_ds, curr_x_dds, curr_x_ebs, curr_ths, curr_th_ds, curr_th_dds, curr_tau_ffs,  curr_tau_fbs);
 



%% CREATE MORE OF THESE 'BIG' MATRICES AND CONCATENATE THEM IN A LOOP
for B = 1:9 %batch size

    % DESIRED TRAJECTORY DATA
    w1 = 70;
    w2 = 80;
    ranw = w1 + rand()*(w2-w1);
    tp.w = ranw*d2r;            % rotational velocity rad/s
    %tp.w = 72*d2r;            % rotational velocity rad/s

    %Randomize radii
    rex1 = 1.7;
    rex2 = 1.8;
    rey1 = 1.2;
    rey2 = 1.3;
    tp.rx = rex1 +rand()*(rex2-rex1); 
    tp.ry = rey1 +rand()*(rey2-rey1);
    %tp.rx = 1.75; tp.ry = 1.25; % ellipse radii

    %Randomize inclination angle
    ell_an1 = 44;
    ell_an2 = 46;
    tp.ell_an = (ell_an1 + rand()*(ell_an2-ell_an1))*d2r;       % angle of inclination of ellipse
    %tp.ell_an = 45*d2r;       % angle of inclination of ellipse

    %randomize center of elipse
    x01 = 0.3;
    x02 = 0.5;
    y01 = 0.3;
    y02 = 0.5;
    tp.x0 = x01 + rand()*(x02-x01);  
    tp.y0 = y01 + rand()*(y02-y01);  % center of ellipse  
    %tp.x0 = 0.4;  tp.y0 = 0.4;  % center of ellipse  

    % Calculate desired trajectory in task space and in joint space
    des = calculate_trajectory(t, tp, rp);
    th_0 = des.th(:,1) - [0.1; 0.2];
    th_d_0 = des.th_d(:,1);

    % SIMULATE ROBOT
    Kp = [500; 500];
    Kd = [50; 50];
    curr = simulate_robot(t, dt, th_0, th_d_0, des, rp, ...
        @(th_curr, th_d_curr, th_des, th_d_des, th_dd_des) ff_dyn_model_1(th_curr, th_d_curr, th_des, th_d_des, th_dd_des, rp), ...
        @(th_curr, th_d_curr, th_des, th_d_des) fb_pd(th_curr, th_d_curr, th_des, th_d_des, Kp, Kd));

	% PUT DATA INTO THE SAME VECTOR
    
    %put each current state in a vector, odd is J1, even J2
    curr_x      = zeros(1,668);
    curr_x_d    = zeros(1,668);
    curr_x_dd   = zeros(1,668);
    curr_x_eb   = zeros(1,668);
    curr_th     = zeros(1,668);
    curr_th_d   = zeros(1,668);
    curr_th_dd  = zeros(1,668);
    curr_tau_ff = zeros(1,668);
    curr_tau_fb = zeros(1,668);
    
    for i = 1:668
        %disp(curr.x(i));
        curr_x(i)      = curr.x(i); %vector of x's: odd is first joint, even is second
        curr_x_d(i)    = curr.x_d(i);
        curr_x_dd(i)   = curr.x_dd(i);
        curr_x_eb(i)   = curr.x_eb(i);
        curr_th(i)     = curr.th(i);
        curr_th_d(i)   = curr.th_d(i);
        curr_th_dd(i)  = curr.th_dd(i);
        curr_tau_ff(i) = curr.tau_ff(i);
        curr_tau_fb(i) = curr.tau_fb(i);
    end
    
    %separate the full vector between even and odd joints
    % odd index values (1st joint)
    curr_x1    = curr_x(1:2:end) ;
    curr_x_d1  = curr_x_d(1:2:end) ;
    curr_x_dd1 = curr_x_dd(1:2:end) ;
    curr_x_eb1 = curr_x_eb(1:2:end);
    curr_th1   = curr_th(1:2:end);
    curr_th_d1 = curr_th_d(1:2:end) ;
    curr_th_dd1 = curr_th_dd(1:2:end) ;
    curr_tau_ff1 = curr_tau_ff(1:2:end) ;
    curr_tau_fb1 = curr_tau_fb(1:2:end) ;
    % even index values (2nd joint)
    curr_x2    = curr_x(2:2:end) ;
    curr_x_d2  = curr_x_d(2:2:end) ;
    curr_x_dd2 = curr_x_dd(2:2:end) ;
    curr_x_eb2 = curr_x_eb(2:2:end);
    curr_th2  = curr_th(2:2:end);
    curr_th_d2 = curr_th_d(2:2:end) ;
    curr_th_dd2 = curr_th_dd(2:2:end) ;
    curr_tau_ff2 = curr_tau_ff(2:2:end) ;
    curr_tau_fb2 = curr_tau_fb(2:2:end) ;
    
    %create a 2x334 matrix for each state (feature)
    
    curr_xs    = [curr_x1;curr_x2];     %should yield the same as curr.x
    curr_x_ds  = [curr_x_d1;curr_x_d2];
    curr_x_dds = [curr_x_dd1;curr_x_dd2];
    curr_x_ebs = [curr_x_eb1;curr_x_eb2];
    curr_ths = [curr_th1;curr_th2];
    curr_th_ds = [curr_th_d1;curr_th_d2];
    curr_th_dds = [curr_th_dd1;curr_th_dd2];
    curr_tau_ffs = [curr_tau_ff1;curr_tau_ff2];
    curr_tau_fbs = [curr_tau_fb1;curr_tau_fb2];
      
    CURR_NEW = cat(3, curr_xs, curr_x_ds, curr_x_dds, curr_x_ebs, curr_ths, curr_th_ds, curr_th_dds, curr_tau_ffs,  curr_tau_fbs);

    %% NOT NEEDED
    %robot_animation(t, curr, des);
    %analyze_performance(t, curr, des);

    %% SAVE ALL STATES PER RUN INTO ONE MAT FILE (ONE FOR EACH RUN)
    %filename = sprintf('File%d.mat', B);
    %save(filename, 'C3');    %each file (1 to batch size) has now a 2x334x9 matrix
     
    CURR_BIG = cat(4, CURR_BIG, CURR_NEW);
end

%Next: stack all together in the 4th dimension

save('BIG.mat', 'CURR_BIG')



    
 