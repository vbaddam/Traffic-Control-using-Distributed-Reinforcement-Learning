import pandas as pd
import numpy as np
import sys
#from A2 import *
#from ac_agent import *
from drl import *
import tensorflow as tf
#import tensorflow.compat.v1 as tf
#tf.disable_v2_behavior()
from keras.backend.tensorflow_backend import set_session
import threading
from threading import Thread, Lock
import time
from keras import backend as K
try:
    sys.path.append('/home/vasanth/Documents/sumo/tools')
    from sumolib import checkBinary
    import traci
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")
'''
config = tf.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.Session(config=config)
set_session(sess)
K.set_session(sess)
'''
#graph = tf.get_default_graph()


class SUMOINT:

    def __init__(self):
        self.lock = Lock()


    def getstates(self, junctionID, lanesID, aID):
        
        positionMatrix = []
        velocityMatrix = []

        cellLength = 7
        offset = 11
        speedLimit = 14

        junctionPosition = traci.junction.getPosition(junctionID[aID])[0]
        vehicles_road1 = traci.edge.getLastStepVehicleIDs(lanesID[aID][0])
        vehicles_road2 = traci.edge.getLastStepVehicleIDs(lanesID[aID][1])
        vehicles_road3 = traci.edge.getLastStepVehicleIDs(lanesID[aID][2])
        vehicles_road4 = traci.edge.getLastStepVehicleIDs(lanesID[aID][3])

        for i in range(12):
            positionMatrix.append([])
            velocityMatrix.append([])
            for _ in range(12):
                positionMatrix[i].append(0)
                velocityMatrix[i].append(0)


        for v in vehicles_road1:
            ind = int(
                abs((junctionPosition - traci.vehicle.getPosition(v)[0] - offset)) / cellLength)
            if(ind < 12):
                positionMatrix[2 - traci.vehicle.getLaneIndex(v)][11 - ind] = 1
                velocityMatrix[2 - traci.vehicle.getLaneIndex(
                    v)][11 - ind] = traci.vehicle.getSpeed(v) / speedLimit

        for v in vehicles_road2:
            ind = int(
                abs((junctionPosition - traci.vehicle.getPosition(v)[0] + offset)) / cellLength)
            if(ind < 12):
                positionMatrix[3 + traci.vehicle.getLaneIndex(v)][ind] = 1
                velocityMatrix[3 + traci.vehicle.getLaneIndex(
                    v)][ind] = traci.vehicle.getSpeed(v) / speedLimit

        junctionPosition = traci.junction.getPosition(junctionID[aID])[1]
        for v in vehicles_road3:
            ind = int(
                abs((junctionPosition - traci.vehicle.getPosition(v)[1] - offset)) / cellLength)
            if(ind < 12):
                positionMatrix[6 + 2 -
                               traci.vehicle.getLaneIndex(v)][11 - ind] = 1
                velocityMatrix[6 + 2 - traci.vehicle.getLaneIndex(
                    v)][11 - ind] = traci.vehicle.getSpeed(v) / speedLimit

        for v in vehicles_road4:
            ind = int(
                abs((junctionPosition - traci.vehicle.getPosition(v)[1] + offset)) / cellLength)
            if(ind < 12):
                positionMatrix[9 + traci.vehicle.getLaneIndex(v)][ind] = 1
                velocityMatrix[9 + traci.vehicle.getLaneIndex(
                    v)][ind] = traci.vehicle.getSpeed(v) / speedLimit
        
        light = []
        if(traci.trafficlight.getPhase(TLS[aID]) == 4):
            light = [1, 0]
        else:
            light = [0, 1]
        

        position = np.array(positionMatrix).reshape(1,12, 12, 1)
        

        velocity = np.array(velocityMatrix).reshape(1, 12, 12, 1)

        
        lights = np.array(light)
        lights = lights.reshape(1, 2, 1)
        

        return [position, velocity, lights]
    
    '''
    def light(self, aID):
        light = []
        if(traci.trafficlight.getPhase(TLS[aID]) == 4):
            light = [1, 0]
        else:
            light = [0, 1]
        
        lights = np.array(light)
        lights = lights.reshape(1, 2, 1)

        return lights
    '''

       

    
    def step(self, TLS, lanesID, aID, light, action, stepz):

        waiting_time = 0
        reward1 = 0
        reward2 = 0

        if(action == 0 and light[0][0][0] == 0):
                # Transition Phase
                for i in range(6):
                    stepz += 1
                    traci.trafficlight.setPhase(TLS[aID], 1)
                    waiting_time += (traci.edge.getLastStepHaltingNumber(lanesID[aID][0]) + traci.edge.getLastStepHaltingNumber(
                        lanesID[aID][1]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][2]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][3]))
                for i in range(10):
                    stepz += 1
                    traci.trafficlight.setPhase(TLS[aID], 2)
                    waiting_time += (traci.edge.getLastStepHaltingNumber(lanesID[aID][0]) + traci.edge.getLastStepHaltingNumber(
                        lanesID[aID][1]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][2]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][3]))
                    traci.simulationStep()
                for i in range(6):
                    stepz += 1
                    traci.trafficlight.setPhase(TLS[aID], 3)
                    waiting_time += (traci.edge.getLastStepHaltingNumber(lanesID[aID][0]) + traci.edge.getLastStepHaltingNumber(
                        lanesID[aID][1]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][2]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][3]))
                    traci.simulationStep()

                # Action Execution
                reward1 = traci.edge.getLastStepVehicleNumber(
                    lanesID[aID][0]) + traci.edge.getLastStepVehicleNumber(lanesID[aID][1])
                reward2 = traci.edge.getLastStepHaltingNumber(
                    lanesID[aID][2]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][3])
                for i in range(10):
                    stepz += 1
                    traci.trafficlight.setPhase(TLS[aID], 4)
                    reward1 = traci.edge.getLastStepVehicleNumber(
                    lanesID[aID][0]) + traci.edge.getLastStepVehicleNumber(lanesID[aID][1])
                    reward2 = traci.edge.getLastStepHaltingNumber(lanesID[aID][2]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][3])
                    waiting_time += (traci.edge.getLastStepHaltingNumber(lanesID[aID][0]) + traci.edge.getLastStepHaltingNumber(
                        lanesID[aID][1]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][2]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][3]))
                    traci.simulationStep()

        if(action == 0 and light[0][0][0] == 1):
                # Action Execution, no state change
            reward1 = traci.edge.getLastStepVehicleNumber(lanesID[aID][0]) + traci.edge.getLastStepVehicleNumber(lanesID[aID][1])
            reward2 = traci.edge.getLastStepHaltingNumber(lanesID[aID][2]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][3])
            for i in range(10):
                stepz += 1
                traci.trafficlight.setPhase(TLS[aID], 4)
                reward1 = traci.edge.getLastStepVehicleNumber(lanesID[aID][0]) + traci.edge.getLastStepVehicleNumber(lanesID[aID][1])
                reward2 = traci.edge.getLastStepHaltingNumber(lanesID[aID][2]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][3])
                waiting_time += (traci.edge.getLastStepHaltingNumber(lanesID[aID][0]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][1]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][2]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][3]))
                traci.simulationStep()

        if(action == 1 and light[0][0][0] == 0):
                # Action Execution, no state change
            reward1 = traci.edge.getLastStepVehicleNumber(
                    lanesID[aID][2]) + traci.edge.getLastStepVehicleNumber(lanesID[aID][3])
            reward2 = traci.edge.getLastStepHaltingNumber(
                    lanesID[aID][0]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][1])
            for i in range(10):
                stepz += 1
                reward1 = traci.edge.getLastStepVehicleNumber(
                    lanesID[aID][2]) + traci.edge.getLastStepVehicleNumber(lanesID[aID][3])
                reward2 = traci.edge.getLastStepHaltingNumber(
                    lanesID[aID][0]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][1])
                waiting_time += (traci.edge.getLastStepHaltingNumber(lanesID[aID][0]) + traci.edge.getLastStepHaltingNumber(
                        lanesID[aID][1]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][2]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][3]))
                traci.simulationStep()

        if(action == 1 and light[0][0][0] == 1):
            for i in range(6):
                stepz += 1
                traci.trafficlight.setPhase(TLS[aID], 5)
                waiting_time += (traci.edge.getLastStepHaltingNumber(lanesID[aID][0]) + traci.edge.getLastStepHaltingNumber(
                        lanesID[aID][1]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][2]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][3]))
                traci.simulationStep()
            for i in range(10):
                stepz += 1
                traci.trafficlight.setPhase(TLS[aID], 6)
                waiting_time += (traci.edge.getLastStepHaltingNumber(lanesID[aID][0]) + traci.edge.getLastStepHaltingNumber(
                        lanesID[aID][1]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][2]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][3]))
                traci.simulationStep()
            for i in range(6):
                stepz += 1
                traci.trafficlight.setPhase(TLS[aID], 7)
                waiting_time += (traci.edge.getLastStepHaltingNumber(lanesID[aID][0]) + traci.edge.getLastStepHaltingNumber(
                        lanesID[aID][1]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][2]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][3]))
                traci.simulationStep()

            reward1 = traci.edge.getLastStepVehicleNumber(
                    lanesID[aID][2]) + traci.edge.getLastStepVehicleNumber(lanesID[aID][3])
            reward2 = traci.edge.getLastStepHaltingNumber(
                    lanesID[aID][0]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][1])
            for i in range(10):
                stepz += 1
                traci.trafficlight.setPhase(TLS[aID], 0)
                reward1 = traci.edge.getLastStepVehicleNumber(
                    lanesID[aID][2]) + traci.edge.getLastStepVehicleNumber(lanesID[aID][3])
                reward2 = traci.edge.getLastStepHaltingNumber(
                    lanesID[aID][0]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][1])
                waiting_time += (traci.edge.getLastStepHaltingNumber(lanesID[aID][0]) + traci.edge.getLastStepHaltingNumber(
                        lanesID[aID][1]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][2]) + traci.edge.getLastStepHaltingNumber(lanesID[aID][3]))
                traci.simulationStep()

        return([reward1, reward2, waiting_time, stepz])

    '''
    def run(self):

        junctionID = ['1', '2']
        lanesID = [['a','e','c','g'],['f','k','i','m']]
        aID = [0,1]
        TLS = ['gneJ15', 'gneJ18']

        sumoInt = SUMOINT()
        sumoBinary = checkBinary('sumo')

        episodes = 10
        batch_size = 32

        agent1 = A2CAgent()
        agent2 = A2CAgent()



        for e in range(episodes):
            step = 0
            waiting_time = 0
            total_reward = 0
            stepz = 0
            action = 0

            traci.start([sumoBinary, "-c", "cross3ltl.sumocfg", '--start'])
            traci.trafficlight.setPhase(TLS[0], 0)
            traci.trafficlight.setPhaseDuration(TLS[0], 200)
            traci.trafficlight.setPhase(TLS[1], 0)
            traci.trafficlight.setPhaseDuration(TLS[1], 200)

            while traci.simulation.getMinExpectedNumber() > 0:

                traci.simulationStep()
                state1 = SUMOINT().getstates(junctionID, lanesID, 0)
                action1 = agent1.act(state1)
                light1 = state1[2]

                state2 = SUMOINT().getstates(junctionID, lanesID, 1)
                action2 = agent2.act(state2)
                light2 = state2[2]

            
                r1, r2, w1, s1 = SUMOINT().step(TLS, lanesID, aID[0], light1, action1, stepz)
                r3, r4, w2, s2 = SUMOINT().step(TLS, lanesID, aID[1], light2, action2, stepz)

                waiting_time = waiting_time + w1 + w2
                stepz = stepz + s1 + s2
                reward1 = r1 - r2
                reward2 = r3 - r4


            
            
                agent1.remember(state1, action1, reward1)
                agent2.remember(state2, action2, reward2)

            
            if(len(agent1.memory) > batch_size):
                update_policy(agent1, reward, log_prob1)
            if(len(agent2.memory) > batch_size):
                update_policy(agent2, reward, log_prob2)
            
            agent1.train()
            agent2.train()


        
        

        #mem1 = agent1.memory[-1]
        #del agent1.memory[-1]
        #agent1.memory.append((mem1[0], mem1[1], reward, mem1[3], True))
        #mem2 = agent2.memory[-1]
        #del agent2.memory[-1]
        #agent2.memory.append((mem2[0], mem2[1], reward, mem2[3], True))
        #log.write('episode - ' + str(e) + ', total waiting time - ' +str(waiting_time))

            print('episode - ' + str(e) + ' total waiting time - ' + str(waiting_time))

        
        #agent.save('reinf_traf_control_' + str(e) + '.h5')
            traci.close(wait=False)
    '''
    def train(self, n_threads):

        junctionID = ['1', '2']
        lanesID = [['a','e','c','g'],['f','k','i','m']]
        aID = [0,1]
        TLS = ['gneJ15', 'gneJ18']
        
        # Instantiate one environment per thread
        #envs = [gym.make(self.env_name) for i in range(n_threads)]

        # Create threads
        threads = []
        for i in range(n_threads):
            t = threading.Thread(target=self.train_threading, args=([i]))
            threads.append(t)

        '''
        threads = [threading.Thread(
                target=self.train_threading,
                args=(self, i)) for i in range(n_threads)]
        '''

        for t in threads:
            time.sleep(2)
            t.start()
            
        for t in threads:
            time.sleep(10)
            t.join()

    def train_threading(self, thread):
        junctionID = ['1', '2']
        lanesID = [['a','e','c','g'],['f','k','i','m']]
        aID = [0,1]
        TLS = ['gneJ15', 'gneJ18']

        sumoInt = SUMOINT()
        sumoBinary = checkBinary('sumo')

        episodes = 100
        batch_size = 64
        rew1 = []
        rew2 = []
        tot = []
        ep = []

        agent = A3CAgent()

        for e in range(episodes):
            step = 0
            waiting_time = 0
            total_reward = 0
            stepz = 0
            action = 0
            re1 = []

            states, actions, rewards = [], [], []

            traci.start([sumoBinary, "-c", "cross3ltl.sumocfg", '--start'])
            #traci.trafficlight.setPhase(TLS[0], 0)
            #traci.trafficlight.setPhaseDuration(TLS[0], 200)
            #traci.trafficlight.setPhase(TLS[1], 0)
            #traci.trafficlight.setPhaseDuration(TLS[1], 200)

            while traci.simulation.getMinExpectedNumber() > 0:

                traci.simulationStep()
                state = SUMOINT().getstates(junctionID, lanesID, thread)
                action = agent.act(state)
                light = state[2]

            

            
                r1, r2, w1, s1 = SUMOINT().step(TLS, lanesID, aID[thread], light, action, stepz)
            

                waiting_time = waiting_time + w1
                stepz = stepz + s1
                reward1 = r1 - r2

                states.append(state)
                action_onehot = np.zeros([2])
                action_onehot[action] = 1
                actions.append(action_onehot)
                rewards.append(reward1)

                re1.append(reward1)
            
            self.lock.acquire()
            agent.replay(states, actions, rewards)
            self.lock.release()
           


            
            
                
            

            '''
            if(len(agent1.memory) > batch_size):
                update_policy(agent1, reward, log_prob1)
            if(len(agent2.memory) > batch_size):
                update_policy(agent2, reward, log_prob2)
            '''
            


        
        

        #mem1 = agent1.memory[-1]
        #del agent1.memory[-1]
        #agent1.memory.append((mem1[0], mem1[1], reward, mem1[3], True))
        #mem2 = agent2.memory[-1]
        #del agent2.memory[-1]
        #agent2.memory.append((mem2[0], mem2[1], reward, mem2[3], True))
        #log.write('episode - ' + str(e) + ', total waiting time - ' +str(waiting_time))

            tot.append(waiting_time)
            ep.append(e)
            rew1.append(np.mean(re1))

            print('episode - ' + str(e) + ' total waiting time - ' + str(waiting_time))
            
        
        #agent.save('reinf_traf_control_' + str(e) + '.h5')
            traci.close(wait=False)
        df = pd.DataFrame(list(zip(ep, tot, rew1)), columns =['episodes', 'total_waiting_time', 'reward'])
        df.to_csv('a2c4.csv')

if __name__ == "__main__":

    junctionID = ['1', '2']
    lanesID = [['a','e','c','g'],['f','k','i','m']]
    aID = [0,1]
    TLS = ['gneJ15', 'gneJ18']
    agent = A3CAgent()
    SUMOINT().train(n_threads=2)
