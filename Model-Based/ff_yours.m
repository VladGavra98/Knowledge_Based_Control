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
     
    
    des = your_parameters{1};
    t   = your_parameters{2};
    net = your_parameters{3};
    
    
%     ind = 1:length(t);
%     
% 
%     state = 1;
%     current_time = ind(des.th_d(state,:) == th_d_des(state));
    
%     current_time = current_time+1;
       

    x = vertcat(th_des, th_d_des, th_dd_des);
    x = num2cell(x,[1,2]);
    
%     debugging:
%     disp(x)
%     disp(net.Layers)
    tau = predict(net,x);
    tau_ff = cell2mat(tau);

    
%    tau_ff = tau_pred{1}(:,current_time);
%    disp(tau_ff);

    % Basic:
%     tau_ff = [0;0];

end