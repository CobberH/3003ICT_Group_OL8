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

# --- Camera ---
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)
W = camera.getWidth()
H = camera.getHeight()
print(f"Camera resolution: {W} x {H}")

# Robot geometry
r = 0.0205
L = 0.052

# Pose estimate
x, y, theta = 0.0, 0.0, 0.0

# State
state = "SEARCH"

# Print control
step_count = 0

# Target memory
targets_found = []
target_visible_last_step = False
TARGET_LIMIT = 3
all_targets_found = False


blue_detected = False
red_detected = False

# Mapping grid and search variables
visited = set()
hazards = set()
red_hazards = []
blue_hazards = []
visit_counts = {}

# Let encoders initialise
robot.step(TIME_STEP)
prevL = leftEnc.getValue()
prevR = rightEnc.getValue()



# Stuck variables
last_x, last_y = 0.0, 0.0
stuck_counter = 0
recovery_step = 0
STUCK_LIMIT = 30

# Visit check
GRID_SIZE = 0.2
REVISIT_LIMIT = 80
search_turn_steps = 0
SEARCH_TURN_TIME = 8

# Colour Check
x_start = W // 3
x_end = 2 * W // 3


def update_odometry(x, y, theta, dL, dR, r, L):
    dl = dL * r
    dr = dR * r
    dc = (dl + dr) / 2.0
    dtheta = (dr - dl) / L

    theta += dtheta
    x += dc * math.cos(theta)
    y += dc * math.sin(theta)

    return x, y, theta

# Note object position instead of robot position
def estimate_object_position(x, y, theta, distance=0.15):
    obj_x = x + distance * math.cos(theta)
    obj_y = y + distance * math.sin(theta)

    return round(obj_x, 2), round(obj_y, 2)

# Prevent identifying same target multiple times
def is_new_detection(new_x, new_y, existing_items, min_dist=0.25):
    for item in existing_items:
        old_x = item[0]
        old_y = item[1]

        dist = math.hypot(new_x - old_x, new_y - old_y)

        if dist < min_dist:
            return False

    return True

# Detection: Red
def detect_red_object(camera, W, H):
    image = camera.getImage()

    red_count = 0
    total_count = 0

    # Check lower/middle area of camera
    for y in range(H // 3, H):
        for x in range(x_start, x_end):
            r = camera.imageGetRed(image, W, x, y)
            g = camera.imageGetGreen(image, W, x, y)
            b = camera.imageGetBlue(image, W, x, y)

            if r > 120 and g < 80 and b < 80:
                red_count += 1

            total_count += 1

    if total_count == 0:
        return False

    red_ratio = red_count / total_count
    return red_ratio > 0.03

# Detection: Green
def detect_green_object(camera, W, H):
    image = camera.getImage()

    green_count = 0
    total_count = 0

    # Check lower/middle area of camera
    for y in range(H // 3, H):
        for x in range(0, W):
            r = camera.imageGetRed(image, W, x, y)
            g = camera.imageGetGreen(image, W, x, y)
            b = camera.imageGetBlue(image, W, x, y)

            if g > 90 and g > r * 1.2 and g > b * 1.2:
                green_count += 1

            total_count += 1

    if total_count == 0:
        return False

    green_ratio = green_count / total_count
    return green_ratio > 0.03

# Detection: Blue
def detect_blue_object(camera, W, H):
    image = camera.getImage()

    blue_count = 0
    total_count = 0

    # Check lower/middle area of camera
    for y in range(H // 3, H):
        for x in range(x_start, x_end):
            r = camera.imageGetRed(image, W, x, y)
            g = camera.imageGetGreen(image, W, x, y)
            b = camera.imageGetBlue(image, W, x, y)

            if r < 80 and g < 80 and b > 120:
                blue_count += 1

            total_count += 1

    if total_count == 0:
        return False

    blue_ratio = blue_count / total_count
    return blue_ratio > 0.03

# Explore: drive forward unless front sensors see obstacle
def search(psValues):
    front_obstacle = psValues[0] > 80.0 or psValues[7] > 80.0

    if front_obstacle:
        return -0.5 * MAX_SPEED, 0.5 * MAX_SPEED
    else:
        return 0.5 * MAX_SPEED, 0.5 * MAX_SPEED

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

    # Stuck check
    movement = math.hypot(x - last_x, y - last_y)

    if movement < 0.001 and state == "SEARCH":
        stuck_counter += 1
    else:
        stuck_counter = 0

    last_x, last_y = x, y

    cell = (
        round(x / GRID_SIZE),
        round(y / GRID_SIZE)
    )

    if cell not in visit_counts:
        visit_counts[cell] = 0

    visit_counts[cell] += 1

    # Camera updates
    target_found = detect_green_object(camera, W, H)
    blue_detected = detect_blue_object(camera, W, H)
    red_detected = detect_red_object(camera, W, H)

    # Detect obstacles
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_blue_obstacle = (psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0) and (blue_detected)
    right_blue_obstacle = (psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0) and (blue_detected)
    left_red_obstacle = (psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0) and (red_detected)
    right_red_obstacle = (psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0) and (red_detected)

    # Decide state
    if stuck_counter > STUCK_LIMIT:
        state = "RECOVERY"
    elif target_found and not target_visible_last_step and len(targets_found) < TARGET_LIMIT:
        state = "TARGET_DETECTED"
    elif left_obstacle or right_obstacle or left_blue_obstacle or right_blue_obstacle or left_red_obstacle or right_red_obstacle:
        state = "SAFETY_NAV"
    elif len(targets_found) >= TARGET_LIMIT:
        state = "ALL FOUND"
    else:
        state = "SEARCH"

    match state:

        case "SEARCH":
            # explore logic
            if search_turn_steps > 0:
                leftSpeed = 0.25 * MAX_SPEED
                rightSpeed = -0.25 * MAX_SPEED
                search_turn_steps -= 1

            elif visit_counts[cell] > REVISIT_LIMIT:
                search_turn_steps = SEARCH_TURN_TIME
                visit_counts[cell] = 0

                leftSpeed = 0.25 * MAX_SPEED
                rightSpeed = -0.25 * MAX_SPEED

            else:
                leftSpeed, rightSpeed = search(psValues)

        case "SAFETY_NAV":
            # immediate obstacle avoidance
            if left_blue_obstacle:
                leftSpeed = 0.5 * MAX_SPEED
                rightSpeed = -0.5 * MAX_SPEED
                hazard_x, hazard_y = estimate_object_position(x, y, theta, distance=0.25)

                if is_new_detection(hazard_x, hazard_y, blue_hazards, min_dist=0.10):
                    blue_hazards.append((hazard_x, hazard_y))
                    hazards.add((hazard_x, hazard_y, "BLUE"))

            elif right_blue_obstacle:
                leftSpeed = -0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED
                hazard_x, hazard_y = estimate_object_position(x, y, theta, distance=0.25)

                if is_new_detection(hazard_x, hazard_y, blue_hazards, min_dist=0.10):
                    blue_hazards.append((hazard_x, hazard_y))
                    hazards.add((hazard_x, hazard_y, "BLUE"))

            elif left_red_obstacle:
                leftSpeed = 0.5 * MAX_SPEED
                rightSpeed = -0.5 * MAX_SPEED
                hazard_x, hazard_y = estimate_object_position(x, y, theta, distance=0.25)

                if is_new_detection(hazard_x, hazard_y, red_hazards, min_dist=0.20):
                    red_hazards.append((hazard_x, hazard_y))
                    hazards.add((hazard_x, hazard_y, "RED"))

            elif right_red_obstacle:
                leftSpeed = -0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED
                hazard_x, hazard_y = estimate_object_position(x, y, theta, distance=0.25)

                if is_new_detection(hazard_x, hazard_y, red_hazards, min_dist=0.20):
                    red_hazards.append((hazard_x, hazard_y))
                    hazards.add((hazard_x, hazard_y, "RED"))

            elif left_obstacle:
                leftSpeed = 0.5 * MAX_SPEED
                rightSpeed = -0.5 * MAX_SPEED

            elif right_obstacle:
                leftSpeed = -0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED

        case "RECOVERY":
            # stuck recovery
            recovery_step += 1

            if recovery_step < 5:
                # stop
                leftSpeed = 0.0
                rightSpeed = 0.0

            elif recovery_step < 20:
                # reverse
                leftSpeed = -0.3 * MAX_SPEED
                rightSpeed = -0.3 * MAX_SPEED

            elif recovery_step < 40:
                # rotate
                leftSpeed = 0.3 * MAX_SPEED
                rightSpeed = -0.3 * MAX_SPEED

            else:
                # recovery complete
                recovery_step = 0
                stuck_counter = 0
                state = "SEARCH"
                leftSpeed = 0.3 * MAX_SPEED
                rightSpeed = 0.3 * MAX_SPEED 

        case "TARGET_DETECTED":
            # confirm target
            leftSpeed = 0.0
            rightSpeed = 0.0

            target_x, target_y = estimate_object_position(x, y, theta, distance=0.25)

            if is_new_detection(target_x, target_y, targets_found, min_dist=0.25):
                targets_found.append((target_x, target_y))
                print(f"TARGET {len(targets_found)} FOUND at x={target_x}, y={target_y}")
            else:
                print("Duplicate target ignored")
                leftSpeed = 0.25 * MAX_SPEED
                rightSpeed = -0.25 * MAX_SPEED

            target_visible_last_step = True

        case "APPROACH_TARGET":
            # move toward survivor
            a = 1 # placeholder

        case "ALL FOUND":
            leftSpeed = 0.0
            rightSpeed = 0.0

            if not all_targets_found:
                print("MISSION COMPLETE: 3 targets found")
                print(f"Targets: {targets_found}")
                print(f"Hazards: {hazards}")
                all_targets_found = True

    grid_x = round(x, 1)
    grid_y = round(y, 1)
    visited.add((grid_x, grid_y))

    # Print pose every 10 steps
    step_count += 1
    if step_count % 10 == 0:
        print(
            f"state={state}, x={x:.2f}, y={y:.2f}, "
            f"theta={math.degrees(theta):.1f}, "
            f"visited={len(visited)}, hazards={len(hazards)}, "
            f"targets={len(targets_found)}"
        )

    if not target_found:
        target_visible_last_step = False

    # Slow down near coloured objects
    if target_found or red_detected or blue_detected:
        leftSpeed *= 0.5
        rightSpeed *= 0.5

    # Apply motor speeds
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
