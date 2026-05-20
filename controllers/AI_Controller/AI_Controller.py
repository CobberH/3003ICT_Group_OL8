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

# blue hazard recovery handling
blue_reverse = 0
blue_timer = 25
blocked_turn_steps = 0
BLOCKED_TURN_TIME = 15



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
blocked_cells = set() # inaccessible terrain

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

    x, y, theta = update_odometry(x, y, theta, dL, dR, r, L)

    # Stuck check
    movement = math.hypot(x - last_x, y - last_y)

    if movement < 0.001 and state == "SEARCH":
        stuck_counter += 1
    else:
        stuck_counter = 0

    last_x, last_y = x, y

    cell = position_to_cell(x, y)

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
    if state == "BLUE_ESCAPE" and blue_reverse > 0:
        state = "BLUE_ESCAPE"
    elif stuck_counter > STUCK_LIMIT:
        state = "RECOVERY"
    elif left_blue_obstacle or right_blue_obstacle:
        state = "BLUE_ESCAPE"
    elif target_found and not target_visible_last_step and len(targets_found) < TARGET_LIMIT:
        state = "TARGET_DETECTED"
    elif left_obstacle or right_obstacle or left_red_obstacle or right_red_obstacle:
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

        #blue recovery
        case "BLUE_ESCAPE":
            blue_reverse += 1
        
            hazard_x, hazard_y = estimate_object_position(x, y, theta, distance=0.25)
        
            if is_new_detection(hazard_x, hazard_y, blue_hazards, min_dist=0.10):
                blue_hazards.append((hazard_x, hazard_y))
                hazards.add((hazard_x, hazard_y, "BLUE"))
        
                hazard_cell = position_to_cell(hazard_x, hazard_y)
                block_hazard_area(blocked_cells, hazard_x, hazard_y, x, y, radius=1)
        
                print(f"BLUE HAZARD marked at x={hazard_x}, y={hazard_y}, cell={hazard_cell}")
        
            if blue_reverse < 10:
                leftSpeed = -0.35 * MAX_SPEED
                rightSpeed = -0.35 * MAX_SPEED
        
            elif blue_reverse < blue_timer:
                leftSpeed = 0.35 * MAX_SPEED
                rightSpeed = -0.35 * MAX_SPEED
        
            else:
                blue_reverse = 0
                search_turn_steps = 0
                state = "SEARCH"
                leftSpeed = 0.4 * MAX_SPEED
                rightSpeed = 0.4 * MAX_SPEED
            
        case "SAFETY_NAV":
            # immediate obstacle avoidance
            if left_blue_obstacle:
                # Turn away from blue/water hazard
                leftSpeed = 0.5 * MAX_SPEED
                rightSpeed = -0.5 * MAX_SPEED

                hazard_x, hazard_y = estimate_object_position(x, y, theta, distance=0.25)

                if is_new_detection(hazard_x, hazard_y, blue_hazards, min_dist=0.10):
                    blue_hazards.append((hazard_x, hazard_y))
                    hazards.add((hazard_x, hazard_y, "BLUE"))

                    hazard_cell = position_to_cell(hazard_x, hazard_y)
                    block_hazard_area(blocked_cells, hazard_x, hazard_y, x, y, radius=1)

                    print(f"BLUE HAZARD marked at x={hazard_x}, y={hazard_y}, cell={hazard_cell}")

            elif right_blue_obstacle:
                # Turn away from blue/water hazard
                leftSpeed = -0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED

                hazard_x, hazard_y = estimate_object_position(x, y, theta, distance=0.25)

                if is_new_detection(hazard_x, hazard_y, blue_hazards, min_dist=0.10):
                    blue_hazards.append((hazard_x, hazard_y))
                    hazards.add((hazard_x, hazard_y, "BLUE"))
        
                    hazard_cell = position_to_cell(hazard_x, hazard_y)
                    block_hazard_area(blocked_cells, hazard_x, hazard_y, radius=1)
        
                    print(f"BLUE HAZARD marked at x={hazard_x}, y={hazard_y}, cell={hazard_cell}")
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
                
                print_discovered_map()
                
                all_targets_found = True

    grid_x = round(x, 1)
    grid_y = round(y, 1)
    visited.add(cell)

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
