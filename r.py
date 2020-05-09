import numpy as np

positionMatrix = []
velocityMatrix = []

for i in range(12):
    positionMatrix.append([])
    velocityMatrix.append([])
    for j in range(12):
        positionMatrix[i].append(0)
        velocityMatrix[i].append(0)


p = np.zeros((12,12))
print(p[1])
print(positionMatrix[1])