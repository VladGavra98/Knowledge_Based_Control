%%%% Dynamics Controller
function tau_ff = ff_yours(th_curr, th_d_curr, th_des, th_d_des, th_dd_des, your_parameters)

%     the robot draws the ellipse approx. twice, consider wrapping the
%     angles to get better training data
%     
%     if you want to use Cartesian positions instead of joint positions in
%     your function approximator:
%     [x_des, x_d_des, x_dd_des, ~] = FK(th_des, th_d_des, th_dd_des, rp);
%     and this is the only purpose for which you are allowed to use the
%     robot parameters rp.
     
    
    net_des = your_parameters{1};
    net_curr = your_parameters{2};

    
    %wrap the angles
    th_des  = wrap(th_des);
    th_curr = wrap(th_curr);
    
    x_curr = vertcat(th_curr,th_d_curr);
    features_curr = length(x_curr);
    x_curr = reshape(x_curr,  [features_curr,1]);
   
    x_des = vertcat(th_des,th_d_des,th_dd_des);
    features_des = length(x_des);
    x_des = reshape(x_des,  [features_des,1]);
    
    % Define fuzzy vairavbles xfz = [delta_1; delta_2]
    xfz = x_curr(1:2)-x_des(1:2);   % difference between current and desired theta

%      Fuzzy stuff - TO DO
    error = sqrt(xfz' * xfz);
    if error >= 0.8
        tau_ff = predict(net_curr,x_curr);
%         disp('curr')
    else
%         disp('des')
        tau_ff = predict(net_des,x_des);
    end
%   
%     % Define clusters and calcaulte memberships:
%     c = 5;    % # clusters
%     m = 3;    % exponent hyperparam.
%     Xc = [[-0.2;-0.1],[0.215;0.165],[-0.315;0.165],[-0.315;-0.365],[0.215;-0.365]];
%     U = mf(xfz,c,Xc,m);
%     
%     [Umax,rule] = max(U);
%     fprintf("mu = [%1.3f,%1.3f] with model %1.0f \n",xfz,rule-1);
% 
%     
%     tau_ff=[0;0];
   
end