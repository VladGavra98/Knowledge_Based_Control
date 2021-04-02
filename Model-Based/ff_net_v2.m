clear
%%                                Load Data
root = pwd;
file_in  = strcat(root,'\Model-Based\IN_C2.mat');
file_out = strcat(root,'\Model-Based\OUT_C2.mat');

load(file_in);
load(file_out);

[m,T] = size(IN);           % number of samples
[features,T] = size(IN{1});

% %                    Change it to image format
% for i=1:1:m
%     IN{i} =  reshape(IN{i},  [1,1,1,features]);
% end
% 
% for i=1:1:m
%     OUT{i} =  reshape(OUT{i},  [1,2]);
% end

%%                        Split Train/Test

p = 0.95;    % ratio between training batch size and testing size
v = 0.05;
idx = randperm(m);


% Switch to cell arrays for later
Xtrain = IN(idx(1:round(p*m)),:,:,:);
Xtest  = IN(idx(round(p*m)+1:end),:,:,:);

Ytrain = OUT(idx(1:round(p*m)),:,:,:);
Ytest  = OUT(idx(round(p*m)+1:end),:,:,:);




disp("Loaded & correct shape")


%%                           PRE-PROCESSING
% mu = mean([Xtrain{:}],[2 3]);
% sig = std([Xtrain{:}],0,[2 3]);
% 
% for i = 1:numel(Xtrain)
%     Xtrain{i} = (Xtrain{i} - mu) ./ sig;
% end
% 
% mu = mean([Xtest{:}],[2 3]);
% sig = std([Xtest{:}],0,[2 3]);
% 
% for i = 1:numel(Xtest)
%     Xtest{i} = (Xtest{i} - mu) ./ sig;
% end
% 
% %%     Save Mu & Sigma
% filename = strcat(root,'\Model-Based\mu_sig.mat');
% mu_sig = {mu, sig};
% save(filename,'mu_sig');


%%                             BUILD MODEL
max_epochs = 80;
mini_batch = 8*256;


num_responses = size(Ytrain{1},1);

validation_freq = 1000;

 % Feedforward deep:  
layers = [ ... 
%     imageInputLayer([1 1 features],'Name','Input_layer')
    sequenceInputLayer(features,'Name','Input_Layer')
    
    fullyConnectedLayer(200,'Name','FC_11')
    reluLayer('Name','relu_11')



    fullyConnectedLayer(200,'Name','FC_12')
    reluLayer('Name','relu_12')
    
    fullyConnectedLayer(100,'Name','FC_13')
    reluLayer('Name','relu_13')
    
    

    fullyConnectedLayer(200,'Name','FC_2')
    reluLayer('Name','relu_2')
    
    dropoutLayer(0.1)

    fullyConnectedLayer(num_responses,'Name','FC_3')
    regressionLayer('Name','Output_layer')];


options = trainingOptions('adam', ...
    'Epsilon',10^(-8),...
    'L2Regularization',0.05,...
    'MaxEpochs',max_epochs, ...
    'MiniBatchSize',mini_batch, ...
    'LearnRateSchedule','piecewise',...
    'InitialLearnRate',0.02, ...
    'LearnRateDropPeriod',5,...
    'LearnRateDropFactor',0.8,...
    'Shuffle','every-epoch', ...
    'ValidationData',{Xtest,Ytest}, ...
    'ValidationFrequency',validation_freq, ...
    'ExecutionEnvironment', 'gpu', ...
    'Shuffle', 'every-epoch',...
    'Plots','training-progress',...
    'Verbose',1);


disp('Model created')
%%                             TRAIN MODEL
disp("Start training ... ")


[model,trainer] = trainNetwork(Xtrain,Ytrain,layers,options);   % continue training
% net = train(net,Xtrain,Ytrain);


disp("Model is trained")
%%                              TEST MODEL

Ypred = predict(model,Xtest,'MiniBatchSize',1000);


 %%                           VERIFICATION
t = 1:1:T;
% Plot a result:
i = randi(round((1-p)*m));

Xtest{i}(1),Ytest{i}(1),Ypred{i}(1)


%%                              ERROR METRICS

figure('Name','Error Metrics: RMSE')
iters = 1:1:length(trainer.TrainingRMSE);
hold on;
% plot(iters(30:30:end),trainer.ValidationRMSE(30:30:end))
plot(iters,trainer.TrainingRMSE)
hold off;


%%                               SAVE MODEL

filename = strcat(root,'\Model-Based\model_trainer_C3.mat');   % C1 = cluster 1
model_trainer = {model, trainer};
save(filename,'model_trainer');


disp("Model is saved to root directory")
%% %%%%%
disp("Done!")
