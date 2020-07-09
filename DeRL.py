
import numpy as np
import pandas as pd
from agent import *
import sys
try:
    sys.path.append('/home/vasanth/Documents/sumo/tools')
    from sumolib import checkBinary
    import traci
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")


class SUMOINT:


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

        position = np.array(positionMatrix).reshape(1, 12, 12, 1)
        

        velocity = np.array(velocityMatrix).reshape(1, 12, 12, 1)


        lights = np.array(light)
        lights = lights.reshape(1, 2, 1)

        return [position, velocity, lights]

    
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


gamma = 0.8

'''
def update_policy(policy_network, rewards, log_probs):
    discounted_rewards = []

    for t in range(len(rewards)):
        Gt = 0 
        pw = 0
        for r in rewards[t:]:
            Gt = Gt + gamma**pw * r
            pw = pw + 1
        discounted_rewards.append(Gt)
        
    discounted_rewards = torch.tensor(discounted_rewards)
    discounted_rewards = (discounted_rewards - discounted_rewards.mean()) / (discounted_rewards.std() + 1e-9) # normalize discounted rewards

    policy_gradient = []
    for log_prob, Gt in zip(log_probs, discounted_rewards):
        policy_gradient.append(-log_prob * Gt)
    
    policy_network.optimizer.zero_grad()
    policy_gradient = torch.stack(policy_gradient).sum()
    policy_gradient.backward()
    policy_network.optimizer.step()
'''


'''
sumoBinary = checkBinary('sumo')
traci.start([sumoBinary, "-c", "cross3ltl.sumocfg", '--start'])

traci.trafficlight.setPhase("0", 0)
traci.trafficlight.setPhaseDuration("0", 200)
j = traci.junction.getPosition('1')[0]
stepz = 0
while traci.simulation.getMinExpectedNumber() > 0 and stepz < 2:
    print(j)
    stepz = stepz + 1


'''
if __name__ == "__main__":
    junctionID = ['1', '2']
    lanesID = [['a','e','c','g'],['f','k','i','m']]
    aID = [0,1]
    TLS = ['gneJ15', 'gneJ18']

    sumoInt = SUMOINT()
    sumoBinary = checkBinary('sumo')

    episodes = 100
    batch_size = 32

    agent1 = DQNAgent()
    agent2 = DQNAgent()
    rew1 = []
    rew2 = []
    tot = []
    ep = []


    for e in range(episodes):
        log = open('wait_time_log.txt', 'a')
        act_f = open('Action of agents.txt', 'a')
        step = 0
        waiting_time = 0
        total_reward = 0
        stepz = 0
        action = 0
        re1 = []
        re2 = []



        traci.start([sumoBinary, "-c", "cross3ltl.sumocfg", '--start'])
        #traci.trafficlight.setPhase(TLS[0], 0)
        #traci.trafficlight.setPhaseDuration(TLS[0], 200)
        #traci.trafficlight.setPhase(TLS[1], 0)
        #traci.trafficlight.setPhaseDuration(TLS[1], 200)

        while traci.simulation.getMinExpectedNumber() > 0:

            traci.simulationStep()
            state1 = SUMOINT().getstates(junctionID, lanesID, 0)
            action1 = agent1.act(state1)
            light1 = state1[2]

            state2 = SUMOINT().getstates(junctionID, lanesID, 1)
            action2 = agent2.act(state2)
            light2 = state1[2]

            act_f.write('Action of agent 1:' + str(action1))
            act_f.write('Action of agent 2:' + str(action2))
            r1, r2, w1, s1 = SUMOINT().step(TLS, lanesID, aID[0], light1, action1, stepz)
            r3, r4, w2, s2 = SUMOINT().step(TLS, lanesID, aID[1], light2, action2, stepz)

            waiting_time = waiting_time + w1 + w2
            #stepz = stepz + s1 + s2


            new_state1 = SUMOINT().getstates(junctionID, lanesID, aID[0])
            new_state2 = SUMOINT().getstates(junctionID, lanesID, aID[1])
            reward1 = r1 - r2 
            reward2 = r3 - r4
            reward = reward1 + reward2 
            agent1.remember(state1, action1, reward, new_state1, False)
            agent2.remember(state2, action2, reward, new_state2, False)

            re1.append(reward)
            #re2.append(reward2)

            #traci.gui.screenshot("View #0", "images/"+str(stepz)+".png")
            # Randomly Draw 32 samples and train the neural network by RMS Prop algorithm
            if(len(agent1.memory) > batch_size):
                agent1.replay(batch_size)
            if(len(agent2.memory) > batch_size):
                agent2.replay(batch_size)
        

        mem1 = agent1.memory[-1]
        del agent1.memory[-1]
        agent1.memory.append((mem1[0], mem1[1], reward1, mem1[3], True))
        mem2 = agent2.memory[-1]
        del agent2.memory[-1]
        agent2.memory.append((mem2[0], mem2[1], reward2, mem2[3], True))
        log.write('episode - ' + str(e) + ', total waiting time - ' +
                  str(waiting_time))

        tot.append(waiting_time)
        ep.append(e)
        rew1.append(np.mean(re1))
        #rew2.append(np.mean(re2))

    
    



        print('episode - ' + str(e) + ' total waiting time - ' + str(waiting_time))

        
        #agent.save('reinf_traf_control_' + str(e) + '.h5')
        traci.close(wait=False)
    df = pd.DataFrame(list(zip(ep, tot, rew1)), columns =['episodes', 'total_waiting_time', 'reward'])
    df.to_csv('DQN4.csv')

sys.stdout.flush()
