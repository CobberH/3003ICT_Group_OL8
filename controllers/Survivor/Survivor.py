from controller import Supervisor
import random

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

print("Supervisor running")

positions = [
    [0.25, 0.25, 0],
    [-0.25, 0.25, 0],
    [0.25, -0.25, 0],
    [-0.25, -0.25, 0],
    [-0.4, 0.4, 0],
    [0.3, 0.45, 0],
    [-0.4, -0.45, 0]
    
]

# Survivors
survivor1 = robot.getFromDef("Survivor")
survivor2 = robot.getFromDef("Survivor2")
survivor3 = robot.getFromDef("Survivor3")


# random position
random.shuffle(positions)

survivor1.getField("translation").setSFVec3f(positions[0])
survivor2.getField("translation").setSFVec3f(positions[1])
survivor3.getField("translation").setSFVec3f(positions[2])

print("Survivors Worked?")

while robot.step(timestep) != -1:
    pass