%% Clear
clc
clearvars
close all
clear;

%% Initialise sim parameters
batch_size = 500;
batch_size = batch_size - 1; 

rp = define_robot_parameters();
sim_time = 10; % simualtion time in seconds
dt = 0.03; % time difference in seconds
t = 0:dt:sim_time;

separate_joints = 0;

%% INITIAL RUN TO MAKE A 3D MATRIX TO WHICH THEN WE CONTACTENATE 
%copied from controller_1
% DESIRED TRAJECTORY DATA
d2r  = pi/180;              % degrees to radians
tp.w = 72*d2r;              % rotational velocity rad/s
tp.rx = 1.75; tp.ry = 1.25; % ellipse radii
tp.ell_an = 45*d2r;         % angle of inclination of ellipse
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
    
des_x      = zeros(1,668);
des_x_d    = zeros(1,668);
des_x_dd   = zeros(1,668);
des_th     = zeros(1,668);
des_th_d   = zeros(1,668);
des_th_dd  = zeros(1,668);

for i = 1:2*length(t)
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
        
        des_x(i)      = des.x(i); %vector of x's: odd is first joint, even is second
        des_x_d(i)    = des.x_d(i);
        des_x_dd(i)   = des.x_dd(i);
        des_th(i)     = des.th(i);
        des_th_d(i)   = des.th_d(i);
        des_th_dd(i)  = des.th_dd(i);
end
%separate the full vector between even and odd joints
% odd index values (1st joint)
curr_x1      = curr_x(1:2:end) ;
curr_x_d1    = curr_x_d(1:2:end) ;
curr_x_dd1   = curr_x_dd(1:2:end) ;
curr_x_eb1   = curr_x_eb(1:2:end);
curr_th1     = curr_th(1:2:end);
curr_th_d1   = curr_th_d(1:2:end) ;
curr_th_dd1  = curr_th_dd(1:2:end) ;
curr_tau_ff1 = curr_tau_ff(1:2:end) ;
curr_tau_fb1 = curr_tau_fb(1:2:end) ;

des_x1     = des_x(1:2:end) ;
des_x_d1   = des_x_d(1:2:end) ;
des_x_dd1  = des_x_dd(1:2:end) ;
des_th1    = des_th(1:2:end);
des_th_d1  = des_th_d(1:2:end) ;
des_th_dd1 = des_th_dd(1:2:end) ;

% even index values (2nd joint)
curr_x2      = curr_x(2:2:end) ;
curr_x_d2    = curr_x_d(2:2:end) ;
curr_x_dd2   = curr_x_dd(2:2:end) ;
curr_x_eb2   = curr_x_eb(2:2:end);
curr_th2     = curr_th(2:2:end);
curr_th_d2   = curr_th_d(2:2:end) ;
curr_th_dd2  = curr_th_dd(2:2:end) ;
curr_tau_ff2 = curr_tau_ff(2:2:end) ;
curr_tau_fb2 = curr_tau_fb(2:2:end) ;

des_x2     = des_x(2:2:end) ;
des_x_d2   = des_x_d(2:2:end) ;
des_x_dd2  = des_x_dd(2:2:end) ;
des_th2    = des_th(2:2:end);
des_th_d2  = des_th_d(2:2:end) ;
des_th_dd2 = des_th_dd(2:2:end) ;
    
%create a 2x334 matrix for each state (feature)
curr_xs      = [curr_x1;curr_x2];     %should yield the same as curr.x
curr_x_ds    = [curr_x_d1;curr_x_d2];
curr_x_dds   = [curr_x_dd1;curr_x_dd2];
curr_x_ebs   = [curr_x_eb1;curr_x_eb2];
curr_ths     = [curr_th1;curr_th2];
curr_th_ds   = [curr_th_d1;curr_th_d2];
curr_th_dds  = [curr_th_dd1;curr_th_dd2];
curr_tau_ffs = [curr_tau_ff1;curr_tau_ff2];
curr_tau_fbs = [curr_tau_fb1;curr_tau_fb2];

des_xs       = [des_x1;des_x2];     %should yield the same as des.x
des_x_ds     = [des_x_d1;des_x_d2];
des_x_dds    = [des_x_dd1;des_x_dd2];
des_ths      = [des_th1;des_th2];
des_th_ds    = [des_th_d1;des_th_d2];
des_th_dds   = [des_th_dd1;des_th_dd2];


%start the 1st 3D matrix
% CURR_IN = vertcat(curr_ths, curr_th_ds, curr_th_dds);
% CURR_OUT = curr_tau_ffs;
% DES_IN  = vertcat(des_ths, des_th_ds, des_th_dds);

IN  = [curr_ths;curr_th_ds; des_ths;des_th_ds;des_th_dds];
OUT = curr_tau_ffs;

%% CREATE MORE OF THESE 'BIG' MATRICES AND CONCATENATE THEM IN A LOOP
for B = 1:batch_size %batch size

    % DESIRED TRAJECTORY DATA
    w1 = 69;
    w2 = 81;
    ranw = w1 + rand()*(w2-w1);
    tp.w = ranw*d2r;            % rotational velocity rad/s
    %tp.w = 72*d2r;            % rotational velocity rad/s

    %Randomize radii
    rex1 = 1.5;
    rex2 = 1.8;
    rey1 = 1.5;
    rey2 = 1.8;
    tp.rx = rex1 +rand()*(rex2-rex1); 
    tp.ry = rey1 +rand()*(rey2-rey1);
    %tp.rx = 1.75; tp.ry = 1.25; % ellipse radii

    %Randomize inclination angle
    ell_an1 = 35;
    ell_an2 = 55;
    tp.ell_an = (ell_an1 + rand()*(ell_an2-ell_an1))*d2r;       % angle of inclination of ellipse
    %tp.ell_an = 45*d2r;       % angle of inclination of ellipse

    %randomize center of elipse
    x01 = -0.1;
    x02 = 0.2;
    y01 = -0.1;
    y02 = 0.2;
    tp.x0 = x01 + rand()*(x02-x01);  
    tp.y0 = y01 + rand()*(y02-y01);  % center of ellipse  
    %tp.x0 = 0.4;  tp.y0 = 0.4;  % center of ellipse  

    % Calculate desired trajectory in task space and in joint space
    des = calculate_trajectory(t, tp, rp);
    th_0 = des.th(:,1) - [0.1; 0.2];
    th_d_0 = des.th_d(:,1);

    
    
    
    curr = simulate_robot(t, dt, th_0, th_d_0, des, rp, ...
        @(th_curr, th_d_curr, th_des, th_d_des, th_dd_des) ff_dyn_model_1(th_curr, th_d_curr, th_des, th_d_des, th_dd_des, rp), ...
        @(th_curr, th_d_curr, th_des, th_d_des) fb_pd(th_curr, th_d_curr, th_des, th_d_des, Kp, Kd));
       
    A = curr.x(1)^2+curr.x(2)^2+rp.l1^2-rp.l2^2+2*rp.l1*curr.x(1);
    B = -4*rp.l1*curr.x(2);
    C = curr.x(1)^2+curr.x(2)^2+rp.l1^2-rp.l2^2-2*rp.l1*curr.x(1);
    
    if (B^2-4*A*C) < 0
       disp("Continue");
    else
       % Add sample to data
       % PUT DATA INTO THE SAME VECTOR
    
        %put each current state in a vector, odd is J1, even J2
        curr_x      = zeros(1,2*length(t));
        curr_x_d    = zeros(1,2*length(t));
        curr_x_dd   = zeros(1,2*length(t));
        curr_x_eb   = zeros(1,2*length(t));
        curr_th     = zeros(1,2*length(t));
        curr_th_d   = zeros(1,2*length(t));
        curr_th_dd  = zeros(1,2*length(t));
        curr_tau_ff = zeros(1,2*length(t));
        curr_tau_fb = zeros(1,2*length(t));

        des_x      = zeros(1,2*length(t));
        des_x_d    = zeros(1,2*length(t));
        des_x_dd   = zeros(1,2*length(t));
        des_th     = zeros(1,2*length(t));
        des_th_d   = zeros(1,2*length(t));
        des_th_dd  = zeros(1,2*length(t));

        for i = 1:2*length(t)
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

            des_x(i)      = des.x(i); %vector of x's: odd is first joint, even is second
            des_x_d(i)    = des.x_d(i);
            des_x_dd(i)   = des.x_dd(i);
            des_th(i)     = des.th(i);
            des_th_d(i)   = des.th_d(i);
            des_th_dd(i)  = des.th_dd(i);
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

        des_x1     = des_x(1:2:end) ;
        des_x_d1   = des_x_d(1:2:end) ;
        des_x_dd1  = des_x_dd(1:2:end) ;
        des_th1    = des_th(1:2:end);
        des_th_d1  = des_th_d(1:2:end) ;
        des_th_dd1 = des_th_dd(1:2:end) ;


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

        des_x2     = des_x(2:2:end) ;
        des_x_d2   = des_x_d(2:2:end) ;
        des_x_dd2  = des_x_dd(2:2:end) ;
        des_th2    = des_th(2:2:end);
        des_th_d2  = des_th_d(2:2:end) ;
        des_th_dd2 = des_th_dd(2:2:end) ;

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


        des_xs       = [des_x1;des_x2];     %should yield the same as des.x
        des_x_ds     = [des_x_d1;des_x_d2];
        des_x_dds    = [des_x_dd1;des_x_dd2];
        des_ths      = [des_th1;des_th2];
        des_th_ds    = [des_th_d1;des_th_d2];
        des_th_dds   = [des_th_dd1;des_th_dd2];

        IN_NEW = vertcat(curr_ths,curr_th_ds, des_ths, des_th_ds, des_th_dds);
        OUT_NEW = curr_tau_ffs;

        % Update the data set
        IN = cat(3, IN, IN_NEW);
        OUT = cat(3, OUT, OUT_NEW);


      end
    
	
end
disp('Data generated')

%% Next: stack all together in the 4th dimension

IN = permute(IN, [3,1,2]);
OUT = permute(OUT, [3,1,2]);

save('C:\Users\vladg\OneDrive\Documents\Master\Q3_SC42050_Knowledge_based_Control\GitHub_practical\KBC_practical\Model-Based\IN.mat', 'IN')
save('C:\Users\vladg\OneDrive\Documents\Master\Q3_SC42050_Knowledge_based_Control\GitHub_practical\KBC_practical\Model-Based\OUT.mat', 'OUT')

 disp('Data saved')
