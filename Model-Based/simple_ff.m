
clear
%%                                Load Data
root = pwd;
file_in  = strcat(root,'\Model-Based\IN.mat');
file_out = strcat(root,'\Model-Based\OUT.mat');

load(file_in);
load(file_out);

[m, features,T] = size(IN);           % number of samples
dt = 0.03;
t = 1:1:T;

for i=1:1:m
    X(:,i) = (IN{i});
    Y(:,i) = (OUT{i});
end



%%                        Split Train/Test

p = 0.9; % ratio between training batch size and testing size
idx = randperm(m);
noise_value = 0.0;   % add noise for training

% Switch to cell arrays for later
Xtrain = X(:,idx(1:round(p*m)));
Xtest  = X(:,idx(round(p*m)+1:end));



Ytrain = Y(:,idx(1:round(p*m)));
Ytest  = Y(:,idx(round(p*m)+1:end));




disp("Loaded & correct shape")
%% __Testing the data:
% figure('Name','Tau_1');
% hold on;
%
% t = 1:1:T;
%
% for i = 1:10:round(p*m)
%     % plot(Xtrain{i}(1,:,1,1), Ytrain{i}(1,:,1,1))  % tau1 against theta curr 1
%     plot(t, Ytrain{i}(1,:,1))
% end
% hold off;


figure('Name','Theta 1');
hold all
for i = 1:10:round(p*m)
    % plot(Xtrain{i}(1,:,1,1), Ytrain{i}(1,:,1,1))  % tau1 against theta curr 1
    plot(t, Xtrain{i}(6,:,1))
    
end
hold off;


%%                           PRE-PROCESSING
mu = mean([Xtrain{:}],[2 3]);
sig = std([Xtrain{:}],0,[2 3]);

for i = 1:numel(Xtrain)
    Xtrain{i} = (Xtrain{i} - mu) ./ sig;
end

mu = mean([Xtest{:}],[2 3]);
sig = std([Xtest{:}],0,[2 3]);

for i = 1:numel(Xtest)
    Xtest{i} = (Xtest{i} - mu) ./ sig;
end

%%                          Save Mu & Sigma
filename = strcat(root,'\Model-Based\mu_sig.mat');
mu_sig = {mu, sig};
save(filename,'mu_sig');


% mu = mean([Ytrain{:}],[2 3]);
% sig = std([Ytrain{:}],0,[2 3]);
%
% for i = 1:numel(Ytrain)
%     Ytrain{i} = (Ytrain{i} - mu) ./ sig;
% end
%
% mu = mean([Ytest{:}],[2 3]);
% sig = std([Ytest{:}],0,[2 3]);
%
% for i = 1:numel(Ytest)
%     Ytest{i} = (Ytest{i} - mu) ./ sig;
% end

%%                             BUILD MODEL

max_epochs = 20;
mini_batch = 256;

input_size = 10;
num_responses = size(2);

numLayers = 3;
numINputs = 10;

net          = cascadeforwardnet(numInputs,numLayers);
net.numInputs = length(Xtrain);


%%                               TRAIN MODEL
disp("Start training ... ")

net = network(10,2,[1;0],[1; 0],[0 0; 1 0],[0 1]);
net.layers{1}.transferFcn = 'tansig';
net.layers{2}.transferFcn = 'logsig';

view(net)




disp("Model is trained")
%%                              TEST MODEL

y_pred = net(x);
perf = perform(net,y,Ytest)

 %%
t = 1:1:T;
% Plot a result:
i = randi(round((1-p)*m));


figure('Name','Motor 2')
hold on
% plot(Xtrain{i}(1,:,1,1), Ytrain{i}(1,:,1,1))  % tau1 against theta curr 1
plot(t, Ypred{i}(2,:)) % tau-1 against theta curr 1
plot(t, Ytest{i}(2,:)) % tau-1 against theta curr
legend('Predictions','Test data')
hold off


figure('Name','Motor 1')
hold on
% plot(Xtrain{i}(1,:,1,1), Ytrain{i}(1,:,1,1))  % tau1 against theta curr 1
plot(t, Ypred{i}(1,:)) % tau-1 against theta curr 1
plot(t, Ytest{i}(1,:)) % tau-1 against theta curr
legend('Predictions','Test data')
hold off

%%                              ERROR METRICS

figure('Name','Error Metrics: RMSE')
iters = 1:1:length(trainer.TrainingRMSE);
hold on;
% plot(iters(30:30:end),trainer.ValidationRMSE(30:30:end))
plot(iters,trainer.TrainingRMSE)
hold off;


%%                               SAVE MODEL

filename = strcat(root,'\Model-Based\model_trainer.mat');
model_trainer = {model, trainer};
save(filename,'model_trainer');


disp("Model is saved to root directory")
%% %%%%%
