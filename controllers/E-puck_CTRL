from controller import Robot
import math

TIME_STEP = 64
MAX_SPEED = 6.28  # e-puck max wheel speed (rad/s)

# Create robot
robot = Robot()

# Distance sensors
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    sensor = robot.getDevice(psNames[i])
    sensor.enable(TIME_STEP)
    ps.append(sensor)

# Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Wheel encoders
leftEnc = robot.getDevice('left wheel sensor')
rightEnc = robot.getDevice('right wheel sensor')
leftEnc.enable(TIME_STEP)
rightEnc.enable(TIME_STEP)

# Robot geometry
r = 0.0205
L = 0.052

# Pose estimate
x, y, theta = 0.0, 0.0, 0.0

# Multiple goals
goals = [(0.5, 0.0), (0.5, 0.4), (0.0, 0.4)]
goal_i = 0

# Print control
step_count = 0

# Let encoders initialise
robot.step(TIME_STEP)
prevL = leftEnc.getValue()
prevR = rightEnc.getValue()

def update_odometry(x, y, theta, dL, dR, r, L):
    dl = dL * r
    dr = dR * r
    dc = (dl + dr) / 2.0
    dtheta = (dr - dl) / L

    theta += dtheta
    x += dc * math.cos(theta)
    y += dc * math.sin(theta)

    return x, y, theta

def goto_goal(x, y, theta, gx, gy, speed=2.0, k=3.0, stop_dist=0.05):
    dx = gx - x
    dy = gy - y
    dist = math.hypot(dx, dy)

    if dist < stop_dist:
        return 0.0, 0.0, True

    desired = math.atan2(dy, dx)
    err = (desired - theta + math.pi) % (2 * math.pi) - math.pi

    vl = speed - k * err
    vr = speed + k * err

    return vl, vr, False

# Main loop
while robot.step(TIME_STEP) != -1:
    # Read distance sensors
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # Odometry update
    curL = leftEnc.getValue()
    curR = rightEnc.getValue()

    dL = curL - prevL
    dR = curR - prevR

    prevL = curL
    prevR = curR

    x, y, theta = update_odometry(x, y, theta, dL, dR, r, L)

    # Go to current goal
    if goal_i < len(goals):
        gx, gy = goals[goal_i]
        leftSpeed, rightSpeed, reached = goto_goal(x, y, theta, gx, gy)

        leftSpeed = max(-MAX_SPEED, min(MAX_SPEED, leftSpeed))
        rightSpeed = max(-MAX_SPEED, min(MAX_SPEED, rightSpeed))

        if reached:
            print(f"Reached goal {goal_i}: {goals[goal_i]}")
            goal_i += 1
    else:
        leftSpeed = 0.0
        rightSpeed = 0.0
        reached = True

    # Detect obstacles
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0

    # Obstacle avoidance overrides goal navigation
    if left_obstacle:
        leftSpeed = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
    elif right_obstacle:
        leftSpeed = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED

    # Print pose every 10 steps
    step_count += 1
    if step_count % 10 == 0:
        if goal_i < len(goals):
            print(f"x={x:.2f}, y={y:.2f}, theta={math.degrees(theta):.1f}, heading to {goals[goal_i]}")
        else:
            print(f"x={x:.2f}, y={y:.2f}, theta={math.degrees(theta):.1f}, all goals reached")

    # Apply motor speeds
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)