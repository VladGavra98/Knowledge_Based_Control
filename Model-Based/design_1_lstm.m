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
%%                        Split Train/Test

p = 0.9; % ratio between training batch size and testing size
idx = randperm(m);
noise_value = 0.0;   % add noise for training

% Switch to cell arrays for later
Xtrain = IN(idx(1:round(p*m)),:,:,:);
Xtrain = correctSize(Xtrain,T,[features],noise_value);

Xtest  = IN(idx(round(p*m)+1:end),:,:,:);
% Xtest = num2cell(Xtest,[2 3]);
Xtest = correctSize(Xtest,T,[features],0.);

Ytrain = OUT(idx(1:round(p*m)),:,:,:);
% Ytrain = num2cell(Ytrain,[2 3]);
Ytrain = correctSize(Ytrain,T,2,0.);

Ytest  = OUT(idx(round(p*m)+1:end),:,:,:);
% Ytest = num2cell(Ytest,[2 3]);
Ytest = correctSize(Ytest,T,2,0.);

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
max_epochs = 100;
mini_batch = 64;


feature_dimension = features;
num_responses = size(Ytrain{1},1);
num_hidden1 = 100;
num_hidden2 = 100;
num_hidden3 = 50;

validation_freq = mini_batch;

layers = [ ...
    sequenceInputLayer(feature_dimension,'Name','Input Layer')
    
    lstmLayer(num_hidden1,'OutputMode','sequence')
    lstmLayer(num_hidden2,'OutputMode','sequence')

    
    %     fullyConnectedLayer(100)
    %     reluLayer()
    
    %     dropoutLayer(0.3)
    
    %     fullyConnectedLayer(100)
    %     reluLayer()
    
    %     dropoutLayer(0.2)
    fullyConnectedLayer(num_responses)
    regressionLayer];


% options = trainingOptions('adam', ...
%     'Epsilon',10^(-8),...
%     'L2Regularization',0.01,...
%     'MaxEpochs',max_epochs, ...
%     'MiniBatchSize',mini_batch, ...
%     'InitialLearnRate',0.01, ...
%     'Shuffle','never', ...
%      'ValidationData',{Xtest,Ytest}, ...
%     'ValidationFrequency',validation_freq, ...
%     'ExecutionEnvironment', 'gpu', ...
%     'Plots','training-progress',...
%     'Verbose',1);

options = trainingOptions('adam', ...
    'Epsilon',10^(-8),...
    'L2Regularization',0.0005,...
    'MaxEpochs',max_epochs, ...
    'MiniBatchSize',mini_batch, ...
    'LearnRateSchedule','piecewise',...
    'InitialLearnRate',0.01, ...
    'LearnRateDropPeriod',10,...
    'LearnRateDropFactor',0.8,...
    'Shuffle','every-epoch', ...
    'ValidationData',{Xtest,Ytest}, ...
    'ValidationFrequency',validation_freq, ...
    'ExecutionEnvironment', 'gpu', ...
    'Plots','training-progress',...
    'Verbose',1);


%%                             TRAIN MODEL
disp("Start training ... ")

% [model,trainer] = trainNetwork(Xtrain,Ytrain,model.Layers,options);   % continue training
[model,trainer] = trainNetwork(Xtrain,Ytrain,layers,options);


disp("Model is trained")
%%                              TEST MODEL

Ypred = predict(model,Xtest,'MiniBatchSize',1);
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
disp("Done!")
