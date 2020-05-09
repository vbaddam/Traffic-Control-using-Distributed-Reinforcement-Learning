import gym
import numpy as np
import keras
from keras.layers import Input, Conv2D, Flatten, Dense
from keras.models import Model
from keras.models import Sequential
from keras.layers import Dense, Reshape, Flatten
from keras.optimizers import Adam
from keras.layers.convolutional import Convolution2D

class PGAgent:

    def __init__(self):
        self.gamma = 0.99
        self.learning_rate = 0.001
        self.states = []
        self.gradients = []
        self.rewards = []
        self.probs = []
        self.model = self.build_model()
        self.model.summary()

    def build_model(self):
        # Neural Net for Deep-Q learning Model
        input_1 = Input(shape=(12, 12, 1))
        x1 = Conv2D(16, (4, 4), strides=(2, 2), activation='relu')(input_1)
        x1 = Conv2D(32, (2, 2), strides=(1, 1), activation='relu')(x1)
        x1 = Flatten()(x1)

 
        input_2 = Input(shape=(12, 12, 1))
        x2 = Conv2D(16, (4, 4), strides=(2, 2), activation='relu')(input_2)
        x2 = Conv2D(32, (2, 2), strides=(1, 1), activation='relu')(x2)
        x2 = Flatten()(x2)

        input_3 = Input(shape=(2, 1))
        x3 = Flatten()(input_3)
    
        x = keras.layers.concatenate([x1, x2, x3])
        x = Dense(128, activation='relu')(x)
        x = Dense(64, activation='relu')(x)
        x = Dense(2, activation='softmax')(x)

        model = Model(inputs=[input_1, input_2, input_3], outputs=[x])
        model.compile(optimizer=keras.optimizers.RMSprop(lr=self.learning_rate), loss='mse')

        return model


    def memorize(self, state, action, prob, reward):
        y = np.zeros(2)
        y[action] = 1
        self.gradients.append(np.array(y).astype('float32') - prob)
        self.states.append(state)
        self.rewards.append(reward)

    def act(self, state):
        #state = state.reshape([1, state.shape[0]])
        aprob = self.model.predict(state)
        self.probs.append(aprob)
        prob = np.array(aprob / np.sum(aprob))
        action = np.random.choice(2, 1, p=np.squeeze(prob))[0]
        return(action, prob)

    def discount_rewards(self, rewards):
        discounted_rewards = np.zeros_like(rewards)
        running_add = 0
        for t in reversed(range(0, rewards.size)):
            if rewards[t] != 0:
                running_add = 0
            running_add = running_add * self.gamma + rewards[t]
            discounted_rewards[t] = running_add
        return discounted_rewards

    def train(self):
        gradients = np.vstack(self.gradients)
        rewards = np.vstack(self.rewards)
        rewards = self.discount_rewards(rewards)
        #reward = (reward - np.mean(rewards)) / (np.std(rewards) + 1e-7)
        gradients *= rewards
        
        #X = np.squeeze(np.vstack(self.states))
        #Y = self.model.predict(self.states)

        #Y = self.probs + self.learning_rate * np.squeeze(np.vstack([gradients]))
        for i in range(len(self.states)):
            X = self.states[i]
            Y = self.probs[i] + self.learning_rate*gradients[i]
            self.model.fit(X, Y)
        #self.model.fit(X, Y)
        self.states, self.probs, self.gradients, self.rewards = [], [], [], []

    def load(self, name):
        self.model.load_weights(name)

    def save(self, name):
        self.model.save_weights(name)

    
    


    

