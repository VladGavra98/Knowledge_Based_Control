
close all

% tau - torques applied to joints
% th - positions of the joints (angles)
% th_d - velocities of the joints (angular velocity)
% th_dd - acceleration of the joints
% _des - desired values (reference)
% _curr - current values (measured)
% ff_ - feedforward
% fb_ - feedback

rp = define_robot_parameters();
sim_time = 10; % simualtion time in seconds
dt = 0.03; % time difference in seconds
t  = 0:dt:sim_time;
T  = length(t);

%% DESIRED TRAJECTORY DATA
d2r   = pi/180;             % degrees to radians
tp.w  = 70*d2r;            % rotational velocity rad/s
tp.rx = 1.75; tp.ry = 1.25; % ellipse radii
tp.ell_an = 45*d2r;       % angle of inclination of ellipse
tp.x0 = 0.4;  tp.y0 = 0.4;  % center of ellipse  

% Calculate desired trajectory in task space and in joint space
des = calculate_trajectory(t, tp, rp);

% th_0 = des.th(:,1) - [0.1; 0.2];
th_0 = des.th(:,1) - [-0.5236; -0.5236];
th_d_0 = des.th_d(:,1);



% Your Code!:
loaded_model = load('model_trainer_v2.mat');
% load_mu_sig  = load('mu_sig.mat');

net = loaded_model.model_trainer{1};
% mu = load_mu_sig.mu_sig{1};
% sig = load_mu_sig.mu_sig{2};

your_parameters = {net};



%% SIMULATE ROBOT
Kp = [500; 500];
Kd = [50; 50];

curr_pred = simulate_robot(t, dt, th_0, th_d_0, des, rp, ...
    @(th_curr, th_d_curr, th_des, th_d_des, th_dd_des) ff_yours(th_curr, th_d_curr, th_des, th_d_des, th_dd_des, your_parameters), ...
    @(th_curr, th_d_curr, th_des, th_d_des) fb_pd(th_curr, th_d_curr, th_des, th_d_des, Kp, Kd));

curr = simulate_robot(t, dt, th_0, th_d_0, des, rp, ...
    @(th_curr, th_d_curr, th_des, th_d_des, th_dd_des) ff_dyn_model_2(th_curr, th_d_curr, th_des, th_d_des, th_dd_des, rp), ...
    @(th_curr, th_d_curr, th_des, th_d_des) fb_pd(th_curr, th_d_curr, th_des, th_d_des, Kp, Kd));


%% verification:
figure(1);
hold on;
plot(t,curr.tau_ff(1,:));
plot(t,curr_pred.tau_ff(1,:));
legend('true','predicted')

% plot(t,tau_pred{1})


%%
robot_animation(t, curr_pred, des);
analyze_performance(t, curr_pred, des);

