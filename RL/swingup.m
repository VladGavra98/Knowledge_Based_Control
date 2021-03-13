function [par, ta, xa] = swingup(par)

    par.simtime = 20;     % Trial length
    par.simstep = 0.05;   % Simulation time step
    par.maxtorque = 1.5;  % Maximum applicable torque
    
    
    if strcmp(par.run_type, 'learn')
        %%
        % Obtain SARSA parameters
        par = get_parameters(par);
        
		% TODO: Initialize the outer loop
        Q = init_Q(par);
        reward =0;
        
        % Initialize book-keeping (for plotting only)
        ra  = zeros(par.trials, 1);
        tta = zeros(par.trials, 1);
        te  = 0;

        % Outer loop: trials
        for ii = 1:par.trials
            
            
            % TODO: Initialize the inner loop
             x = swingup_initial_state();   % zero angle and zero velocity
             a = randi(5);                  % random initial torque
             
            % Inner loop: simulation steps
            for tt = 1:ceil(par.simtime/par.simstep)   % K time iterations
                
                % TODO: obtain torque
                u = take_action(a, par);
                s = discretize_state(x, par);
                
                % Apply torque and obtain new state
                % x  : state (input at time t and output at time t+par.simstep)
                % u  : torque
                % te : new time
                [te, x] = body_straight([te te+par.simstep],x,u,par);

                % TODO: learn
                % use s for discretized state
                sP = discretize_state(x, par);
                
                aP = execute_policy(Q, sP, par);

                reward = observe_reward(a, sP, par);

                Q = update_Q(Q, s, a, reward, sP, aP, par);
                a = aP;

                %debugging:
%                 a,Q(s(1),s(2),a)  %problem with update
                
            
                % Keep track of cumulative reward
                ra(ii) = ra(ii)+reward;

                % TODO: check termination condition
                % Stop trial if state is terminal
                if is_terminal(sP, par)
                    break
                end
            end

            tta(ii) = tta(ii) + tt*par.simstep;

            % Update plot every ten trials
            if rem(ii, 10) == 0
                plot_Q(Q, par, ra, tta, ii);
                drawnow;
            end
            
            % Annealing:
%             par.epsilon = par.epsilon/1.001
        end
        
        % save learned Q value function
        par.Q = Q;
 
    elseif strcmp(par.run_type, 'test')
        %%
        % Obtain SARSA parameters
        par = get_parameters(par);
        
        % Read value function
        Q = par.Q;
        
        x = swingup_initial_state();
        
        ta = zeros(length(0:par.simstep:par.simtime), 1);
        xa = zeros(numel(ta), numel(x));
        te = 0;
        
        % Initialize a new trial
        s = discretize_state(x, par);
        a = execute_policy(Q, s, par);

        % Inner loop: simulation steps
        for tt = 1:ceil(par.simtime/par.simstep)
            % Take the chosen action
            TD = max(min(take_action(a, par), par.maxtorque), -par.maxtorque);

            % Simulate a time step
            [te,x] = body_straight([te te+par.simstep],x,TD,par);

            % Save trace
            ta(tt) = te;
            xa(tt, :) = x;

            s = discretize_state(x, par);
            a = execute_policy(Q, s, par);
            
            % Stop trial if state is terminal
            if is_terminal(s, par)
                break
            end
            
        end
        disp('Time steps taken to learn')
        disp(tt)
        ta = ta(1:tt);
        xa = xa(1:tt, :);
        
    elseif strcmp(par.run_type, 'verify')
        %%
        % Get pointers to (decorators of) functions
        learner.get_parameters = @get_parameters;
        learner.init_Q = @init_Q;
        learner.discretize_state = @discretize_state;
        learner.execute_policy = @execute_policy;
        learner.observe_reward = @observe_reward;
        learner.is_terminal = @is_terminal;
        learner.update_Q = @update_Q;
        learner.take_action = @take_action;
        par.learner = learner;
    end
    
end

% ******************************************************************
% ***                Edit below this line                        ***
% ******************************************************************

function par = get_parameters(par)
    % TODO: set the values
    par.epsilon = 0.2;        % Random action rate      CHANGED FROM 0
    par.epsilon0 = 0.2;       %For plotting only!!
    par.gamma = 1;       % Discount rate
    par.alpha = 0.25;          % Learning rate           CHANGED FROM 0
    par.pos_states = 31;     % Position discretization   CHANGED FROM 0
    par.vel_states = 31;     % Velocity discretization   CHANGED FROM 0
    par.actions = 5;        % Action discretization
    par.trials = 2000;         % Learning trials
end

function Q = init_Q(par)
    % TODO: Initialize the Q table.
    Q = 1*ones(par.pos_states,par.vel_states,par.actions);

    % optimistic search (even if we go rather greedy later)
end

function s = discretize_state(x, par)
    % TODO: Discretize state. Note: s(1) should be
    % TODO: position, s(2) velocity.
    s(1) = floor(mod(x(1),2*pi)*par.pos_states/(2*pi)) + 1 ;
    
    x(2) = max(min(x(2),5*pi),-5*pi);
    s(2) = floor(x(2)*(par.vel_states-1)/(10*pi) + par.vel_states/2 +1)   ;
   
end

function u = take_action(a, par)
    % TODO: Calculate the proper torque for action a. This cannot
    % TODO: exceed par.maxtorque.
    
   u = a*2*par.maxtorque/ (par.actions-1) - 3*par.maxtorque/2;
end

function r = observe_reward(a, sP, par)
    % TODO: Calculate the reward for taking action a,
    % TODO: resulting in state sP.
    
    % Reward proportional to the potential energy, independent of action a
    r(sP(1) == floor(par.pos_states/2) && sP(2) == floor(par.vel_states/2)) = 10;
    r(sP(1) ~= floor(par.pos_states/2) || sP(2) ~= floor(par.vel_states/2)) = 0;


end

function t = is_terminal(sP, par)
    % TODO: Return 1 if state sP is terminal, 0 otherwise.
%     if sP(1) ==15 && sP(2) ==15
%         t =1;
%     else
%         t=0;
%     end
    t(sP(1) == floor(par.pos_states/2) && sP(2) == floor(par.vel_states/2)) = 1;
    t(sP(1) ~= floor(par.pos_states/2) || sP(2) ~= floor(par.vel_states/2)) = 0 ;
    
end


function a = execute_policy(Q, s, par)
    % TODO: Select an action for state s using the
    % TODO: epsilon-greedy algorithm.
    
    p = rand;
    if p <= par.epsilon
         a = randi(par.actions);      % exploration 
    else
        [Qmax,a] = max(Q(s(1),s(2),:));    % greedy  
    end
end

function Q = update_Q(Q, s, a, r, sP, aP, par)
    % TODO: Implement the SARSA update rule.
%     disp('Boot-strap');  Q(sP(1),sP(2), aP) - Q(s(1),s(2), a)
    Q(s(1),s(2),a) = Q(s(1),s(2),a) + par.alpha .* (r + par.gamma .* Q(sP(1),sP(2), aP) - Q(s(1),s(2), a)) ;
    
end

