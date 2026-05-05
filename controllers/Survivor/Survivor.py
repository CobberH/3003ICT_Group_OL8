from controller import Supervisor
import random

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

print("Supervisor running")

# Object must have DEF TARGET in the world
target = robot.getFromDef("Survivor")

if target is None:
    print("ERROR: Could not find object with DEF TARGET")
else:
    translation = target.getField("translation")

    positions = [
        [0.5, 0.5, 0.0],
        [-0.5, 0.5, 0.0],
        [0.5, -0.5, 0.0],
        [-0.5, -0.5, 0.0]
    ]

    chosen_position = random.choice(positions)
    translation.setSFVec3f(chosen_position)

    print("Moved TARGET to:", chosen_position)

while robot.step(timestep) != -1:
    pass