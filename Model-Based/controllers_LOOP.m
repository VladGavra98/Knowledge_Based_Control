%% Clear
clc
clearvars
close all
clear;

%% Initialise sim parameters
m = 5000*334;


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


%New: build the 2D cell matrix: each
% IN{i}: 

IN  = {};
OUT = {};

w1 = 69.0;
w2 = 81.0;

x01 = 0.4;
x02 = 0.4;
y01 = 0.4;
y02 = 0.4;

rex1 = 1.74;
rex2 = 1.76;
rey1 = 1.24;
rey2 = 1.26;

ell_an1 = 44;
ell_an2 = 46;   


%% CREATE MORE OF THESE 'BIG' MATRICES AND CONCATENATE THEM IN A LOOP

maux = 1;
for B = 1:floor(m/length(t)) % number of simulations

    % DESIRED TRAJECTORY DATA
    ranw = w1 + rand()*(w2-w1);
    tp.w = ranw*d2r;            % rotational velocity rad/s
    %tp.w = 72*d2r;             % rotational velocity rad/s

    %Randomize radii
    tp.rx = rex1 +rand()*(rex2-rex1);
    tp.ry = rey1 +rand()*(rey2-rey1);
    %tp.rx = 1.75; tp.ry = 1.25; % ellipse radii

    %Randomize inclination angle
    tp.ell_an = (ell_an1 + rand()*(ell_an2-ell_an1))*d2r;       % angle of inclination of ellipse
    %tp.ell_an = 45*d2r;       % angle of inclination of ellipse

    %randomize center of elipse

    tp.x0 = x01 + rand()*(x02-x01);  
    tp.y0 = y01 + rand()*(y02-y01);  % center of ellipse  
    %tp.x0 = 0.4;  tp.y0 = 0.4;  % center of ellipse  

    
    %randomize initial deviation:
    angle_deviation = (rand(2,1)*2 -1) .* [32*pi/180 ; 32*pi/180];

    % Calculate desired trajectory in task space and in joint space
    des = calculate_trajectory(t, tp, rp);
    th_0 = des.th(:,1) - angle_deviation;
    th_d_0 = des.th_d(:,1);


    % Simualte robot trajectory:
    curr = simulate_robot(t, dt, th_0, th_d_0, des, rp, ...
        @(th_curr, th_d_curr, th_des, th_d_des, th_dd_des) ff_dyn_model_2(th_curr, th_d_curr, th_des, th_d_des, th_dd_des, rp), ...
        @(th_curr, th_d_curr, th_des, th_d_des) fb_pd(th_curr, th_d_curr, th_des, th_d_des, Kp, Kd));
       
   
       % Add sample to data
        curr_x      = curr.x; % vector of x's: odd is first joint, even is second
        curr_x_d    = curr.x_d;
        curr_x_dd   = curr.x_dd;
        curr_x_eb   = curr.x_eb;
        curr_th     = wrap(curr.th);   % wrap the angle to [0,2pi]
        curr_th_d  = curr.th_d;
        curr_th_dd  = curr.th_dd;
        curr_tau_ff = curr.tau_ff;
        curr_tau_fb = curr.tau_fb;

        des_x      = des.x;   % vector of x's: odd is first joint, even is second
        des_x_d    = des.x_d;
        des_x_dd   = des.x_dd;
        des_th    =  wrap(des.th);    % wrap the angle to [0,2pi]
        des_th_d   = des.th_d;
        des_th_dd  = des.th_dd;
    
       
    for i = 1:length(t)
        
        %state_s has both motor values in it
        curr_xs    = [curr_x(1,i),curr_x(2,i)];     
        curr_x_ds  = [curr_x_d(1,i),curr_x_d(2,i)];
        curr_x_dds = [curr_x_dd(1,i),curr_x_dd(2,i)];
        curr_x_ebs = [curr_x_eb(1,i),curr_x_eb(2,i)];
        curr_ths = [curr_th(1,i),curr_th(2,i)];
        curr_th_ds = [curr_th_d(1,i),curr_th_d(2,i)];
        curr_th_dds = [curr_th_dd(1,i),curr_th_dd(2,i)];
        curr_tau_ffs = [curr_tau_ff(1,i),curr_tau_ff(2,i)];
        curr_tau_fbs = [curr_tau_fb(1,i),curr_tau_fb(2,i)];


        des_xs       = [des_x(1,i),des_x(2,i)];     %should yield the same as des.x
        des_x_ds     = [des_x_d(1,i),des_x_d(2,i)];
        des_x_dds    = [des_x_dd(1,i),des_x_dd(2,i)];
        des_ths      = [des_th(1,i),des_th(2,i)];
        des_th_ds    = [des_th_d(1,i),des_th_d(2,i)];
        des_th_dds   = [des_th_dd(1,i),des_th_dd(2,i)];

        IN_NEW  = horzcat(curr_ths,curr_th_ds,des_ths, des_th_ds, des_th_dds);
        OUT_NEW = curr_tau_ffs;

        % Update the data set
        IN{maux,1}  =  IN_NEW;
        OUT{maux,1} =  OUT_NEW;
        
        maux = maux+1; % update data point

    end
    
	
end

if maux == m+1
    disp('Data generated')
end

%%  Next: stack all together in the 3th dimension

for i=1:1:m
    IN{i} =  reshape(IN{i},  [10,1]);
end

for i=1:1:m
    OUT{i} =  reshape(OUT{i},  [2,1]);
end


%%
save('C:\Users\vladg\OneDrive\Documents\Master\Q3_SC42050_Knowledge_based_Control\GitHub_practical\KBC_practical\Model-Based\IN.mat', 'IN')
save('C:\Users\vladg\OneDrive\Documents\Master\Q3_SC42050_Knowledge_based_Control\GitHub_practical\KBC_practical\Model-Based\OUT.mat', 'OUT')

 disp('Data saved')
