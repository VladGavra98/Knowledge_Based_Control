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
     
    
    net = your_parameters{1};
%     mu = your_parameters{2};
%     sig = your_parameters{3};

    %wrap the angle
    th_des = wrap(th_des);
    
    x = vertcat(th_curr,th_d_curr,th_des, th_d_des, th_dd_des);
    x = reshape(x,  [10,1]);
   
%     x = (x- mu )./ sig;
    tau_ff = predict(net,x);
 

    
%    tau_ff = tau_pred{1}(:,current_time);
%    disp(tau_ff);

    % Basic:
%     tau_ff = [0;0];

end