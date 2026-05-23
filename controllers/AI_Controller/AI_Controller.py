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
leftSpeed = 0.0
rightSpeed = 0.0

# Print control
step_count = 0

# Target memory
targets_found = []
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
blocked_cells = set() # inaccessible terrain

# Let encoders initialise
robot.step(TIME_STEP)
prevL = leftEnc.getValue()
prevR = rightEnc.getValue()

# Object approach and memory
pending_object = None        # None, "GREEN", "RED", "BLUE"
pending_seen_x = 0.0
pending_seen_y = 0.0
pending_seen_theta = 0.0

# Approach parameters
approach_lost_counter = 0
APPROACH_SPEED = 0.25 * MAX_SPEED
APPROACH_LOST_LIMIT = 30
REACQUIRE_TURN_SPEED = 0.2 * MAX_SPEED
CENTER_TOLERANCE_RATIO = 0.12
GREEN_TURN_GAIN = 0.04

# Duplicate distances
GREEN_DUPLICATE_DIST = 0.55
RED_DUPLICATE_DIST = 0.35
BLUE_DUPLICATE_DIST = 0.35
target_ignore_counter = 0
TARGET_IGNORE_TIME = 100

# Sensor threshold for confirming object location
GREEN_CONFIRM_RATIO = 0.10
GREEN_CONFIRM_THRESHOLD = 79.0
RED_CONFIRM_THRESHOLD = 78.0
BLUE_CONFIRM_THRESHOLD = 77.0

# Stuck variables
last_x, last_y = 0.0, 0.0
stuck_counter = 0
recovery_mode = None # None, "STUCK", "BLUE_ESCAPE"
recovery_step = 0
safety_step = 0
SAFETY_ESCAPE_STEPS = 15
safety_escape_counter = 0
last_safety_leftSpeed = -2.0
last_safety_rightSpeed = 2.0
SAFETY_REVERSE_TIME = 10
SAFETY_TURN_TIME = 25
STUCK_LIMIT = 30
OBSTACLE_THRESHOLD = 80.0

# Visit check
GRID_SIZE = 0.2
REVISIT_LIMIT = 80
search_turn_steps = 0
SEARCH_TURN_TIME = 8

# Colour Check
x_start = W // 3
x_end = 2 * W // 3


def update_odometry(x, y, theta, dL, dR, r, L, front_blocked, rear_blocked, leftSpeed, rightSpeed):
    dl = dL * r
    dr = dR * r

    dc = (dl + dr) / 2.0
    dtheta = (dr - dl) / L

    # Prevent fake forward or reverse movement while blocked
    if front_blocked and leftSpeed > 0 and rightSpeed > 0:
        dc = 0.0
    elif rear_blocked and leftSpeed < 0 and rightSpeed < 0:
        dc = 0.0

    theta += dtheta

    x += dc * math.cos(theta)
    y += dc * math.sin(theta)

    return x, y, theta

# Work out new angle to approach target if lost target due to obstacle avoidance
def normalise_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

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

            if r > 140 and r > g * 1.8 and r > b * 1.8:
                red_count += 1

            total_count += 1

    if total_count == 0:
        return False

    red_ratio = red_count / total_count
    return red_ratio > 0.08

# Detection: Green
def analyse_green_object(camera, W, H):
    image = camera.getImage()

    green_count = 0
    total_count = 0
    green_x_sum = 0

    for y in range(H // 3, H):
        for x in range(0, W):
            r = camera.imageGetRed(image, W, x, y)
            g = camera.imageGetGreen(image, W, x, y)
            b = camera.imageGetBlue(image, W, x, y)

            if g > 90 and g > r * 1.2 and g > b * 1.2:
                green_count += 1
                green_x_sum += x

            total_count += 1

    if green_count == 0 or total_count == 0:
        return False, 0.0, None, False

    green_ratio = green_count / total_count
    green_center_x = green_x_sum / green_count

    centre_low = W * 0.35
    centre_high = W * 0.65
    green_centered = centre_low <= green_center_x <= centre_high

    return green_ratio > 0.03, green_ratio, green_center_x, green_centered

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

# Explore and drive forward unless front sensors see obstacle
def search(psValues):
    front_obstacle = psValues[0] > 80.0 or psValues[7] > 80.0

    if front_obstacle:
        return -0.5 * MAX_SPEED, 0.5 * MAX_SPEED
    else:
        return 0.5 * MAX_SPEED, 0.5 * MAX_SPEED

# grid mapping     
def position_to_cell(x, y):
    return (
        round(x / GRID_SIZE),
        round(y / GRID_SIZE)
    )
# water blocked area
def block_hazard_area(blocked_cells, hazard_x, hazard_y, robot_x, robot_y, radius=1):
    hx, hy = position_to_cell(hazard_x, hazard_y)
    robot_cell = position_to_cell(robot_x, robot_y)

    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            new_cell = (hx + dx, hy + dy)

            # avoids blocking robot cell
            if new_cell != robot_cell:
                blocked_cells.add(new_cell)

# How close to get to coloured objects
def is_front_close(pending_object, psValues):
    if pending_object == "GREEN":
        return psValues[0] > GREEN_CONFIRM_THRESHOLD or psValues[7] > GREEN_CONFIRM_THRESHOLD

    elif pending_object == "RED":
        return psValues[0] > RED_CONFIRM_THRESHOLD or psValues[7] > RED_CONFIRM_THRESHOLD

    elif pending_object == "BLUE":
        return psValues[0] > BLUE_CONFIRM_THRESHOLD or psValues[7] > BLUE_CONFIRM_THRESHOLD

    return False

# Obstacle avoidance
def obstacle_avoid(psValues, fwd=2.5, turn=2.0, threshold=80.0):
    front_left = max(psValues[5], psValues[6], psValues[7])
    front_right = max(psValues[0], psValues[1], psValues[2])

    if front_right > threshold:
        return -turn, turn

    elif front_left > threshold:
        return turn, -turn

    return fwd, fwd

def print_discovered_map():
    map_cells = set(visited) # collects all cell data

    target_cells = set() #target cell local
    for tx, ty in targets_found:
        target_cells.add(position_to_cell(tx, ty))

    hazard_cells = set() # hazard cell local
    for hx, hy, colour in hazards:
        hazard_cells.add(position_to_cell(hx, hy))

    all_cells = map_cells | target_cells | hazard_cells

    if not all_cells:
        print("Map is empty")
        return

    min_x = min(c[0] for c in all_cells)
    max_x = max(c[0] for c in all_cells)
    min_y = min(c[1] for c in all_cells)
    max_y = max(c[1] for c in all_cells)

    print("\nDISCOVERED MAP")
    print("O = explored, X = target, H = hazard")
    print()

    for gy in range(max_y, min_y - 1, -1):
        row = ""

        for gx in range(min_x, max_x + 1):
            cell = (gx, gy)

            if cell in target_cells: #target mark
                row += " X "
            elif cell in hazard_cells: #hazard mark
                row += " H "
            elif cell in map_cells: #normal cell mark
                row += " O "
            else:
                row += " . " #undiscovered

        print(row)

    print()

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

    # check if directions are blocked
    front_blocked = psValues[0] > OBSTACLE_THRESHOLD or psValues[7] > OBSTACLE_THRESHOLD
    left_blocked  = psValues[5] > OBSTACLE_THRESHOLD or psValues[6] > OBSTACLE_THRESHOLD
    right_blocked = psValues[1] > OBSTACLE_THRESHOLD or psValues[2] > OBSTACLE_THRESHOLD
    rear_blocked  = psValues[3] > OBSTACLE_THRESHOLD or psValues[4] > OBSTACLE_THRESHOLD

    # update robot position
    x, y, theta = update_odometry(x, y, theta, dL, dR, r, L, front_blocked, rear_blocked, leftSpeed, rightSpeed)

    if target_ignore_counter > 0:
        target_ignore_counter -= 1

    # Stuck check
    movement = math.hypot(x - last_x, y - last_y)

    # robot wheel motors are moving forward - true or false
    forward = (dL + dR) / 2.0 > 0

    # increase stuck counter to enter recovery state
    if (movement < 0.001 and forward and state in ["SEARCH", "APPROACH_OBJECT"] and state != "ALL FOUND"):
        stuck_counter += 1
    else:
        stuck_counter = 0

    # update on whether robot has visited the area
    last_x, last_y = x, y

    cell = position_to_cell(x, y)

    if cell not in visit_counts:
        visit_counts[cell] = 0

    visit_counts[cell] += 1

    # Camera updates
    target_found, green_ratio, green_center_x, green_centered = analyse_green_object(camera, W, H)
    blue_detected = detect_blue_object(camera, W, H)
    red_detected = detect_red_object(camera, W, H)

    # Detect obstacles
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_blue_obstacle = (psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0) and (blue_detected)
    right_blue_obstacle = (psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0) and (blue_detected)
    left_red_obstacle = (psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0) and (red_detected)
    right_red_obstacle = (psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0) and (red_detected)

    # Memory of colour seen
    if target_found and len(targets_found) < TARGET_LIMIT and target_ignore_counter == 0:
        pending_object = "GREEN"
        pending_seen_x = x
        pending_seen_y = y
        pending_seen_theta = theta

    front_close = is_front_close(pending_object, psValues)

    # Make sure the main object on screen is green and robot is close enough to target
    close_pending_object = (pending_object == "GREEN" and front_close and target_found and green_ratio > GREEN_CONFIRM_RATIO and green_centered)

    # Decide state
    if len(targets_found) >= TARGET_LIMIT:
        state = "ALL FOUND"

    elif state == "RECOVERY" and recovery_mode is not None:
        state = "RECOVERY"

    elif stuck_counter > STUCK_LIMIT:
        recovery_mode = "STUCK"
        recovery_step = 0
        state = "RECOVERY"

    elif left_blue_obstacle or right_blue_obstacle:
        recovery_mode = "BLUE_ESCAPE"
        state = "RECOVERY"

    elif close_pending_object:
        state = "APPROACH_OBJECT"

    elif left_obstacle or right_obstacle or left_red_obstacle or right_red_obstacle:
        state = "SAFETY_NAV"

    elif pending_object is not None:
        state = "APPROACH_OBJECT"

    else:
        state = "SEARCH"

    match state:

        case "SEARCH":
            # explore logic
            if front_blocked:
                recovery_mode = "STUCK"
                recovery_step = 0
                state = "RECOVERY"
                leftSpeed = 0.0
                rightSpeed = 0.0
            elif search_turn_steps > 0:
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
            hazard_x = round(x, 2)
            hazard_y = round(y, 2)

            # hazard logging
            if blue_detected:
                if is_new_detection(hazard_x, hazard_y, blue_hazards, min_dist=BLUE_DUPLICATE_DIST):
                    blue_hazards.append((hazard_x, hazard_y))
                    hazards.add((hazard_x, hazard_y, "BLUE"))
                    block_hazard_area(blocked_cells, hazard_x, hazard_y, x, y, radius=1)
                    print(f"BLUE HAZARD marked at x={hazard_x}, y={hazard_y}")

            elif red_detected:
                if is_new_detection(hazard_x, hazard_y, red_hazards, min_dist=RED_DUPLICATE_DIST):
                    red_hazards.append((hazard_x, hazard_y))
                    hazards.add((hazard_x, hazard_y, "RED"))
                    print(f"RED HAZARD marked at x={hazard_x}, y={hazard_y}")

            avoid_left, avoid_right = obstacle_avoid(psValues)

            if avoid_left != 2.5 or avoid_right != 2.5:
                leftSpeed = avoid_left
                rightSpeed = avoid_right

                last_safety_leftSpeed = leftSpeed
                last_safety_rightSpeed = rightSpeed
                safety_escape_counter = SAFETY_ESCAPE_STEPS

            elif safety_escape_counter > 0:
                leftSpeed = last_safety_leftSpeed
                rightSpeed = last_safety_rightSpeed
                safety_escape_counter -= 1

            else:
                leftSpeed = 2.5
                rightSpeed = 2.5

        case "RECOVERY":

            if recovery_mode == "BLUE_ESCAPE":
                recovery_step += 1

                hazard_x = round(x, 2)
                hazard_y = round(y, 2)

                if is_new_detection(hazard_x, hazard_y, blue_hazards, min_dist=BLUE_DUPLICATE_DIST):
                    blue_hazards.append((hazard_x, hazard_y))
                    hazards.add((hazard_x, hazard_y, "BLUE"))
                    block_hazard_area(blocked_cells, hazard_x, hazard_y, x, y, radius=1)

                    print(f"BLUE HAZARD marked at x={hazard_x}, y={hazard_y}")

                if recovery_step < 10:
                    leftSpeed = -0.35 * MAX_SPEED
                    rightSpeed = -0.35 * MAX_SPEED

                elif recovery_step < 30:
                    leftSpeed = -0.5 * MAX_SPEED
                    rightSpeed = -0.5 * MAX_SPEED

                elif recovery_step < 70:
                    leftSpeed = 0.5 * MAX_SPEED
                    rightSpeed = -0.5 * MAX_SPEED

                else:
                    recovery_step = 0
                    recovery_mode = None
                    state = "SEARCH"
                    leftSpeed = 0.4 * MAX_SPEED
                    rightSpeed = 0.4 * MAX_SPEED
                    if pending_object == "BLUE":
                        pending_object = None
                        approach_lost_counter = 0

            # if robot stuck, determine how it is stuck on what sides and steps to get out
            elif recovery_mode == "STUCK":
                recovery_step += 1

                if recovery_step < 5:
                    leftSpeed = 0.0
                    rightSpeed = 0.0

                elif front_blocked and not rear_blocked:
                    # back away if front is blocked
                    leftSpeed = -0.35 * MAX_SPEED
                    rightSpeed = -0.35 * MAX_SPEED

                elif left_blocked and not right_blocked:
                    # turn right away from left obstacle
                    leftSpeed = 0.35 * MAX_SPEED
                    rightSpeed = -0.35 * MAX_SPEED

                elif right_blocked and not left_blocked:
                    # turn left away from right obstacle
                    leftSpeed = -0.35 * MAX_SPEED
                    rightSpeed = 0.35 * MAX_SPEED

                elif recovery_step < 30:
                    # default reverse
                    leftSpeed = -0.3 * MAX_SPEED
                    rightSpeed = -0.3 * MAX_SPEED

                elif recovery_step < 55:
                    # default turn
                    leftSpeed = 0.3 * MAX_SPEED
                    rightSpeed = -0.3 * MAX_SPEED

                else:
                    recovery_step = 0
                    stuck_counter = 0
                    recovery_mode = None
                    state = "SEARCH"
                    leftSpeed = 0.3 * MAX_SPEED
                    rightSpeed = 0.3 * MAX_SPEED

            else:
                recovery_mode = None
                recovery_step = 0
                state = "SEARCH"
                leftSpeed = 0.0
                rightSpeed = 0.0

        case "APPROACH_OBJECT":
            front_close = is_front_close(pending_object, psValues)

            object_visible = (pending_object == "GREEN" and target_found)

            if object_visible:
                approach_lost_counter = 0

                # Seeing if all green checks are present to be determined "green target"
                # Checks if it is 1. close by sensors 2. green dominate 3. green centred
                if (front_close and green_ratio > GREEN_CONFIRM_RATIO and green_centered):
                    obj_x = round(x, 2)
                    obj_y = round(y, 2)

                    leftSpeed = 0.0
                    rightSpeed = 0.0

                    # Found green target if not duplicate
                    if is_new_detection(obj_x, obj_y, targets_found, min_dist=GREEN_DUPLICATE_DIST):
                        targets_found.append((obj_x, obj_y))

                        target_ignore_counter = TARGET_IGNORE_TIME
                        search_turn_steps = SEARCH_TURN_TIME

                        print(f"TARGET {len(targets_found)} FOUND at x={obj_x}, y={obj_y}")

                    # Ignore green target if found before
                    else:
                        target_ignore_counter = TARGET_IGNORE_TIME
                        search_turn_steps = SEARCH_TURN_TIME

                        print("Duplicate target ignored")

                    pending_object = None
                    approach_lost_counter = 0
                    state = "SEARCH"

                # If all checks aren't true, then try and keep green centred and adjust steering to keep headings towards it
                else:
                    if green_center_x is not None:
                        image_center = W / 2
                        error = green_center_x - image_center

                        CENTER_TOLERANCE = W * CENTER_TOLERANCE_RATIO

                        # Check error and correct direction
                        if abs(error) > CENTER_TOLERANCE:
                            turn = 0.015 * error

                            base = 0.20 * MAX_SPEED

                            leftSpeed = base + turn
                            rightSpeed = base - turn

                            leftSpeed = max(-0.4 * MAX_SPEED, min(0.4 * MAX_SPEED, leftSpeed))
                            rightSpeed = max(-0.4 * MAX_SPEED, min(0.4 * MAX_SPEED, rightSpeed))
                        else:
                            leftSpeed = APPROACH_SPEED
                            rightSpeed = APPROACH_SPEED

                    else:
                        leftSpeed = 0.25 * MAX_SPEED
                        rightSpeed = -0.25 * MAX_SPEED

            # If green target lost due to obstacle avoidance, attempt to find green target again
            else:
                approach_lost_counter += 1

                if approach_lost_counter > APPROACH_LOST_LIMIT:
                    print(f"Lost sight of {pending_object}, returning to SEARCH")

                    pending_object = None
                    approach_lost_counter = 0
                    state = "SEARCH"

                    leftSpeed = 0.25 * MAX_SPEED
                    rightSpeed = -0.25 * MAX_SPEED

                else:
                    # turn back toward original sighting direction
                    turn_error = normalise_angle(pending_seen_theta - theta)

                    if abs(turn_error) > 0.15:

                        if turn_error > 0:
                            leftSpeed = -REACQUIRE_TURN_SPEED
                            rightSpeed = REACQUIRE_TURN_SPEED

                        else:
                            leftSpeed = REACQUIRE_TURN_SPEED
                            rightSpeed = -REACQUIRE_TURN_SPEED

                    else:
                        leftSpeed = 0.25 * MAX_SPEED
                        rightSpeed = -0.25 * MAX_SPEED

        case "ALL FOUND":
            leftSpeed = 0.0
            rightSpeed = 0.0

            if not all_targets_found:
                print("MISSION COMPLETE: 3 targets found")
                print(f"Targets: {targets_found}")
                print(f"Hazards: {hazards}")
                
                print_discovered_map()

                all_targets_found = True

    # update cell visisted
    grid_x = round(x, 1)
    grid_y = round(y, 1)
    visited.add(position_to_cell(x, y))

    # Print pose every 10 steps
    step_count += 1
    if step_count % 10 == 0:
        print(
            f"state={state}, x={x:.2f}, y={y:.2f}, "
            f"theta={math.degrees(theta):.1f}, "
            f"visited={len(visited)}, hazards={len(hazards)}, "
            f"targets={len(targets_found)}"
        )

    # Apply motor speeds
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
