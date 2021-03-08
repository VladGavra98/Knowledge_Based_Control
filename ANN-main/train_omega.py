# -*- coding: utf-8 -*-
"""
Created on Sun Mar  7 21:42:18 2021

@author: vladg
"""
from support_fns import seed_experiment
import tensorflow as tf
import tensorflow_addons as tfa
from tensorflow import keras
from tensorflow.keras import layers
import numpy as np
import matplotlib.pyplot as plt
import h5py
import gym
import wrappers
from driver import Driver
#from environments.pendulum import UnderactuatedPendulum
import statistics
import tqdm



# SEED EXPERIMENT TO CREATE REPRODUCIBLE RESULTS
seed_value = 1
seed_experiment(seed=seed_value)  # Comment this line to disable the seeding



### SETUP (untouched)
# USE GPU IF AVAILABLE
gpus = tf.config.experimental.list_physical_devices('GPU')
if len(gpus) > 0:
    tf.config.experimental.set_memory_growth(gpus[0], True)
    print("CUDA is active")
else:
    print("Sad CPU")

# LOAD
with h5py.File('./dataset.h5', 'r') as hf:
    observation = hf['observation.h5'][:]
    state = hf['state.h5'][:]
print('Loaded observation data: %s' % str(observation.shape))
print('Loaded state data: %s' % str(state.shape))


#testing ....
N = 120    # transitions
T = 101    # time steps per transition

# DATASET PARAMETERS
num_examples = N * T #12000

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                                      # DATA PRE-PRO
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

observation = observation.reshape(N,T,28,28,3)   # N x T x 28 x 28 x3
observation = observation[:num_examples]



state = state[:num_examples]
theta = state[:,0]
theta = np.where(theta < 0.0, theta + 2*np.pi, theta)
state[:,0] = theta


# Scale pixel values to a range of 0 to 1 before feeding them to the neural network model.
observation = observation.astype(np.float32) / 255.

# CREATE TEST DATASET
test_split = 0.2
if 0 < test_split < 1:
    split_at = int(len(observation) * (1 - test_split))
else:
    raise ValueError('Must hold-out data as a test dataset. Set parameter 0 < test_split < 1.')

test_obs   = observation[split_at:, :, :, :, :]
test_theta = state[split_at:, 0]
test_trig  = np.hstack([np.sin(test_theta)[:, None], np.cos(test_theta)[:, None]])
test_omega = state[split_at:, :]


# CREATE TRAINING DATASET
train_obs   = observation[:split_at, :, :, :]
train_theta = state[:split_at, 0]
train_trig  = np.hstack([np.sin(train_theta)[:, None], np.cos(train_theta)[:, None]])
train_omega = state[:split_at, :]

# VERIFY TRAINING DATA
# To verify that the data is in the correct format and that you're ready to build and train the network,
# let's display the first 25 images from the dataset and display the corresponding theta value below each image.
# plt.figure(figsize=(10, 10))
# for i in range(25):
#     plt.subplot(5, 5, i+1)
#     plt.xticks([])
#     plt.yticks([])
#     plt.grid(False)
#     plt.imshow(train_obs[i])
#     plt.xlabel(str(round(train_theta[i]/np.pi, 2)) + '$\\pi$')


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                                          # MODELS
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

print("Build model...")
# MODEL PARAMETERS
model_type = 'model_lstm'  # 'model_theta', 'model_trig', 'model_cnn'

# BUILD MODELS
# Building the neural network requires configuring the layers of the model, then compiling the model.

if model_type == 'model_lstm':

    """However, it can be very useful when building a Sequential model incrementally to be able
    to display the summary of the model so far, including the current output shape.
    In this case, you should start your model by passing an Input object to your model,
    so that it knows its input shape from the start:

    model = keras.Sequential()
    model.add(keras.Input(shape=(4,)))
    model.add(layers.Dense(2, activation="relu"))
   """
    model = keras.Sequential()
    model.add(keras.layers.InputLayer(input_shape=(observation.shape[1:])))

    model.add(layers.Conv2D(filters=32,kernel_size=3,activation="relu"))
    model.add(layers.MaxPooling3D(pool_size=(1,2,2)))

    model.add(layers.Conv2D(filters=16,kernel_size=5,activation="relu"))
    model.add(layers.MaxPooling3D(pool_size=(1,3,3)))

    model.add(layers.Flatten()   )


    model.add(layers.Reshape((-1,1)))
    model.add(layers.Dense(T))

    model.add(layers.LSTM(2))

    str_model_type = "LSTM"

else:
    raise ValueError('Unknown model type selected.')

#            VERIFY THAT A MODEL DEFINED
try:
    model
except NameError:
    raise ValueError("Variable 'model' not defined! Make sure to name every keras model 'model'!")

#                COMPILE MODEL


print("Compile model...")
model.compile(optimizer='ADAM',loss='mse' )



#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                                          # FIT MODEL
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# initialize tqdm callback with default parameters
tqdm_callback = tfa.callbacks.TQDMProgressBar()


model.summary()
if model_type == 'model_lstm':
    model.fit(train_obs,train_omega,batch_size=64,epochs=30,shuffle=True)


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                                          # EVALUATE
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


if model_type == 'model_lstm':
    test_scores = model.evaluate(test_obs, test_omega, verbose=2)
    predictions  = model.predict(test_obs)[:, :]

else:
    raise ValueError("Variable 'model' not defined! Make sure to name every keras model 'model'!")

print("Test loss:", test_scores)


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                                         # EVALUATE MODEL ACCURACY
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
test_accuracy = True

if test_accuracy:
    # Calculate average error per bin over theta range [-pi, pi]
    test_error = np.abs(test_theta - predictions[0])
    test_error[test_error > np.pi] -= 2*np.pi
    test_error = np.abs(test_error)
    bins = np.linspace(-np.pi, np.pi, 21)
    digitized = np.digitize(test_theta, bins)
    bin_means = np.array([test_error[digitized == i].mean() for i in range(1, len(bins))])
    fig, ax = plt.subplots()
    ax.bar(bins[:-1], bin_means, width=np.diff(bins), edgecolor="black", align="edge")
    ax.set_xlabel('$\\theta$ (rad)')
    ax.set_ylabel('$|\\bar{\\theta} -\\theta|$ (rad)')
    ax.set_title('%s - Average prediction error %s' % (str_model_type, '{:.2e}'.format(test_error.mean())))



# MODEL SUMMARY
model.summary()
plt.show()


#run multiple times, record manually. For my run, the sample array rec of recorded average prediction errors is:
rec11 = [2.29,2.29,0.214,2.29,0.388,0.194,0.328,2.29,0.216,2.29]
rec12 = [0.0137,0.0148,0.854,0.0112,0.462,0.854,0.00935,0.854,0.0138,0.0198]
rec13 = [0.00123,0.00307,0.00153,0.00294,0.00263,0.00299,0.00174,0.0019,0.00129,0.00221]
#mean and standard deviation of the recorded average errors
mu11 = statistics.mean(rec11)
mu12 = statistics.mean(rec12)
mu13 = statistics.mean(rec13)
sigma11 = statistics.stdev(rec11)
sigma12 = statistics.stdev(rec12)
sigma13 = statistics.stdev(rec13)



### BELOW THIS ONLY SIMULATION (not needed)
# # INITIALIZE TEST_PENDULUM ENVIRONMENT
# num_sims = 20
# len_time = 100
# test_env = gym.make('pendulum-underactuated-v0', render_shape=(500, 500), model=model)
# test_env = wrappers.NormalizeActions(test_env)
# test_env = wrappers.MinimumDuration(test_env, len_time)
# test_env = wrappers.MaximumDuration(test_env, len_time)
# test_env = wrappers.ObservationDict(test_env, key='observation')
# test_env = wrappers.PixelObservations(test_env, (28, 28), np.uint8, 'image')
# test_env = wrappers.ConvertRewardToCost(test_env)
# test_env = wrappers.ConvertTo32Bit(test_env)
# test_env.seed(seed=seed_value)
#
# # INITIALIZE INFRASTRUCTURE
# test_driver = Driver(test_env)
#
# # RUN SIMULATOR
# print('Test trained model in %d simulations.' % num_sims)
# print('Press "ctrl + c" to interrupt the simulation.')
# test_driver.run(render=True, num_steps=len_time*num_sims)
