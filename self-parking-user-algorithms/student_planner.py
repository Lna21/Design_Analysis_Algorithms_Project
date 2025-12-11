"""학생 자율주차 알고리즘 스켈레톤 모듈.

이 파일만 수정하면 되고, 네트워킹/IPC 관련 코드는 `ipc_client.py`에서
자동으로 처리합니다. 학생은 아래 `PlannerSkeleton` 클래스나 `planner_step`
함수를 원하는 로직으로 교체/확장하면 됩니다.
"""

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

# Car Dimensions
CAR_LENGTH = 4.5
CAR_WIDTH = 1.8
CAR_SAFETY_MARGIN = 1.0

# Constants for the Y centers of the two alleys
ALLEY_1_Y_CENTER = 17.0 #17.35 
ALLEY_2_Y_CENTER = 33 #36.65

def pretty_print_map_summary(map_payload: Dict[str, Any]) -> None:
    extent = map_payload.get("extent") or [None, None, None, None]
    slots = map_payload.get("slots") or []
    occupied = map_payload.get("occupied_idx") or []
    free_slots = len(slots) - sum(1 for v in occupied if v)
    print("[algo] map extent :", extent)
    print("[algo] total slots:", len(slots), "/ free:", free_slots)
    stationary = map_payload.get("grid", {}).get("stationary")
    if stationary:
        rows = len(stationary)
        cols = len(stationary[0]) if stationary else 0
        print("[algo] grid size  :", rows, "x", cols)

#Checks if a point is within a rectangle
def point_in_rect(x: float, y: float, rect: List[float]) -> bool:
    if len(rect) < 4:
        return False
    xmin, xmax, ymin, ymax = rect[0], rect[1], rect[2], rect[3]
    return xmin <= x <= xmax and ymin <= y <= ymax

#Checks Collisions
def check_side_collision(car_x: float, car_y: float, car_yaw: float, side: str,
                         slots: List, occupied_idx: List, target_slot: List[float],
                         walls_rects: List) -> bool:


    half_length = CAR_LENGTH / 2.0           # Calculate half dimensions
    half_width = CAR_WIDTH / 2.0            
    margin = CAR_SAFETY_MARGIN               # Set safety margin
    cos_yaw = math.cos(car_yaw)              # Pre-calculate trig functions for rotation
    sin_yaw = math.sin(car_yaw)
    test_points = []                         # Initialize list for collision detection points
    
    if side == 'left':
        for i in range(5):                   # Left side of the car
            t = (i / 4.0) - 0.5              # Normalized position along the car's length (-0.5=rear, 0.5=front)
            local_x = t * CAR_LENGTH
            local_y = half_width + margin    # Y-offset 
            global_x = car_x + local_x * cos_yaw - local_y * sin_yaw  # rotation and translation for X
            global_y = car_y + local_x * sin_yaw + local_y * cos_yaw
            test_points.append((global_x, global_y))
    
    elif side == 'right':
        for i in range(5):
            t = (i / 4.0) - 0.5              # Right side 
            local_x = t * CAR_LENGTH
            local_y = -(half_width + margin) # Y-offset neg
            global_x = car_x + local_x * cos_yaw - local_y * sin_yaw
            global_y = car_y + local_x * sin_yaw + local_y * cos_yaw
            test_points.append((global_x, global_y))
    
    elif side == 'front':
        for i in range(5):                     # Front side detection points
            t = (i / 4.0) - 0.5  
            local_x = half_length + margin * 2 # half-length + extended margin
            local_y = t * CAR_WIDTH            # position along the car's width
            global_x = car_x + local_x * cos_yaw - local_y * sin_yaw
            global_y = car_y + local_x * sin_yaw + local_y * cos_yaw
            test_points.append((global_x, global_y))
    
    elif side == 'front_left':                 # Front-left corner detection point
        local_x = half_length + margin
        local_y = half_width + margin
        global_x = car_x + local_x * cos_yaw - local_y * sin_yaw
        global_y = car_y + local_x * sin_yaw + local_y * cos_yaw
        test_points.append((global_x, global_y))
        
    elif side == 'front_right':                # Front-right corner detection point
        local_x = half_length + margin
        local_y = -(half_width + margin)
        global_x = car_x + local_x * cos_yaw - local_y * sin_yaw
        global_y = car_y + local_x * sin_yaw + local_y * cos_yaw
        test_points.append((global_x, global_y))

    elif side == 'rear':                       # rear side detection points
        for i in range(5):
            t = (i / 4.0) - 0.5  
            local_x = -(half_length + margin * 2)            
            local_y = t * CAR_WIDTH
            global_x = car_x + local_x * cos_yaw - local_y * sin_yaw
            global_y = car_y + local_x * sin_yaw + local_y * cos_yaw
            test_points.append((global_x, global_y))   
        
    elif side == 'rear_left':
        local_x = -(half_length + margin)
        local_y = half_width + margin
        global_x = car_x + local_x * cos_yaw - local_y * sin_yaw
        global_y = car_y + local_x * sin_yaw + local_y * cos_yaw
        test_points.append((global_x, global_y))
        
    elif side == 'rear_right':
        local_x = -(half_length + margin)
        local_y = -(half_width + margin)
        global_x = car_x + local_x * cos_yaw - local_y * sin_yaw
        global_y = car_y + local_x * sin_yaw + local_y * cos_yaw
        test_points.append((global_x, global_y))

    # Convert target_slot list to a tuple for comparison
    target_rect = None
    if target_slot and len(target_slot) >= 4:
        target_rect = tuple(map(float, target_slot))
    
    # Check collision with occupied slots
    for idx, slot in enumerate(slots):
        # Skip if slot definition is incomplete
        if len(slot) < 4:
            continue

        # Extract slot rectangle coordinates
        slot_rect = list(map(float, slot[:4]))
        # Skip the target slot
        is_target = (target_rect and 
                    abs(slot_rect[0] - target_rect[0]) < 0.1 and
                    abs(slot_rect[1] - target_rect[1]) < 0.1 and
                    abs(slot_rect[2] - target_rect[2]) < 0.1 and
                    abs(slot_rect[3] - target_rect[3]) < 0.1)
        
        if is_target:
            continue

        # Only check occupied slots
        if idx < len(occupied_idx) and occupied_idx[idx]:
            for px, py in test_points:
                # If any detection point is inside the occupied slot --> collision imminent
                if point_in_rect(px, py, slot_rect):
                    return True

    # Check collision with environment walls
    if walls_rects:
        for rect in walls_rects:
            for px, py in test_points:
                if point_in_rect(px, py, rect):
                    # Wall collision detected
                    # Return True immediately if any point hits any wall
                    return True 
    return False

@dataclass
class PlannerSkeleton:
    """경로 계획/제어 로직을 담는 기본 스켈레톤 클래스입니다."""

    map_data: Optional[Dict[str, Any]] = None
    map_extent: Optional[Tuple[float, float, float, float]] = None
    cell_size: float = 0.5
    stationary_grid: Optional[List[List[float]]] = None
    waypoints: List[Tuple[float, float]] = None
    expected_orientation: Optional[str] = None
    debug_counter: int = 0
    
    # Topological Navigation State
    navigation_phase: str = "init"
    target_line: int = 0
    turn_y: float = 0.0
    slots: List = None
    occupied_idx: List = None

    # Parking logic fields
    parking_type: int = 0         # 1 or 2 (front_in), 3 (rear_in)
    final_gear: str = "D"         # 'D' for Drive (Forward), 'R' for Reverse (Backward)
    
    # Obstacle Detection Counters
    left_collision_count: int = 0
    right_collision_count: int = 0
    front_collision_count: int = 0
    front_left_collision_count: int = 0
    front_right_collision_count: int = 0
    rear_left_collision_count: int = 0
    rear_right_collision_count: int = 0

    def __post_init__(self) -> None:
        # Initialize waypoints list (dataclass default)
        if self.waypoints is None:
            self.waypoints = []

    def set_map(self, map_payload: Dict[str, Any]) -> None:
        """시뮬레이터에서 전송한 정적 맵 데이터를 보관합니다."""
        self.map_data = map_payload
        
        # Retrieve expected parking orientation
        self.expected_orientation = map_payload.get("expected_orientation", "front_in")
        
        # Deduce PARKING TYPE based on orientation
        if self.expected_orientation == "rear_in":
            parking_designation = "Parking 3 (Reverse Logic)"
            self.parking_type = 3
        else:
            # If 'front_in', it could be Parking 1 or 2
            parking_designation = "Parking 1 or 2"
            self.parking_type = 1 
        
        # Retrieve full map name
        map_name = map_payload.get("name", "Nom de carte inconnu")
        print("-" * 40)
        print(f" PARKING DÉTECTÉ : {parking_designation}")
        
        # Store map properties from payload
        self.map_extent = tuple(
            map(float, map_payload.get("extent", (0.0, 0.0, 0.0, 0.0))))
        self.cell_size = float(map_payload.get("cellSize", 0.5))
        self.stationary_grid = map_payload.get("grid", {}).get("stationary")
        self.expected_orientation = map_payload.get("expected_orientation", "front_in")
        self.slots = map_payload.get("slots", [])
        self.occupied_idx = map_payload.get("occupied_idx", [])
        self.walls_rects = map_payload.get("walls_rects", [])
        
        pretty_print_map_summary(map_payload)
        print(f"[algo] expected_orientation: {self.expected_orientation}")
        print(f"[algo] Loaded {len(self.slots)} slots, {sum(self.occupied_idx)} occupied")
        
        # Reset planner state for a new session
        self.waypoints.clear()
        self.debug_counter = 0
        self.navigation_phase = "init"

        # Reset all collisions counters
        self.left_collision_count = 0
        self.right_collision_count = 0
        self.front_collision_count = 0
        self.front_left_collision_count = 0  
        self.front_right_collision_count = 0 
        self.rear_collision_count = 0
        self.rear_left_collision_count = 0   
        self.rear_right_collision_count = 0         


    def compute_path(self, obs: Dict[str, Any]) -> None:
        if self.navigation_phase != "init":
            return
        
        target_slot = obs.get("target_slot")
        # Ensure valid target slot is provided
        if not target_slot or len(target_slot) < 4:
            return
        
        # Debug print: raw coordinates
        print(f"[algo] TARGET SLOT RAW: {target_slot}")
        slot_xmin, slot_xmax, slot_ymin, slot_ymax = map(float, target_slot)
        # Calculate slot center coordinates
        target_center_x = (slot_xmin + slot_xmax) / 2.0
        target_center_y = (slot_ymin + slot_ymax) / 2.0
        
        # Ensure map boundaries are loaded
        if not self.map_extent:
            return
        
        xmin, xmax, ymin, ymax = self.map_extent
        map_height = ymax - ymin
        line_height = map_height / 3.0
        
        # 1. Determine target parking Row (Target Line)
        # Row mapping: 3=Bottom Row (R1), 1=Top Row (R3).
        if target_center_y < ymin + line_height:
            self.target_line = 3 
        elif target_center_y < ymin + 2 * line_height:
            self.target_line = 2 
        else:
            self.target_line = 1 
        
        # Debug: Convert internal line ID (3, 2, 1) to user nomenclature (1, 2, 3)
        line_mapping = {3: 1, 2: 2, 1: 3}
        display_line = line_mapping.get(self.target_line, self.target_line)
        
        print(f"[algo] TARGET ROW DETECTED : Row {display_line}")
        
        # 2. Determine target Alley Y and Turn Point (self.turn_y)
        # Alley logic: Row 3 -> Alley 1, Rows 2/1 -> Alley 2
        if self.target_line == 3:                            # Corresponds to the lowest row
            alley_y = ALLEY_1_Y_CENTER 
            print("[algo] Chosen Alley: Alley 1".format(ALLEY_1_Y_CENTER))
        else:                                                # self.target_line is 2 or 1
            alley_y = ALLEY_2_Y_CENTER 
            print("[algo] Chosen Alley: Alley ".format(ALLEY_2_Y_CENTER))
        
        self.turn_y = alley_y                                # Y coordinate for the initial turn

        # Set final gear based on parking type (D=1,2; R=3)                       
        if self.parking_type in (1, 2):
            self.final_gear = "D" 
        elif self.parking_type == 3:
            self.final_gear = "R" 
        else:
            self.final_gear = "D"                         # Default to Forward

        # Get current car position
        state = obs.get("state", {})
        car_x = float(state.get("x", 0.0))
        car_y = float(state.get("y", 0.0))
        self.waypoints.clear()

        # FORWARD PARKING LOGIC (P1/P2) 
        if self.final_gear == "D":

            is_far_left_slot = target_center_x < 14.0 
            # Waypoint Creation
            if self.parking_type in (1, 2):
                self.final_gear = "D"
                if is_far_left_slot:
                    # WP 1 Align car to alley Y (slight X-offset)
                    Y_ALIGNMENT_X = car_x + 1.5 
                    self.waypoints.append((Y_ALIGNMENT_X, self.turn_y))
                    pass
                    
                else:
                    # WP 1 Align car to alley Y
                    Y_ALIGNMENT_X = car_x + 1.0 
                    self.waypoints.append((Y_ALIGNMENT_X, self.turn_y))
                    # WP 2 Pre-position point before the turn
                    PRE_POSITION_X = target_center_x - CAR_LENGTH *0.8 #0.9 
                    self.waypoints.append((PRE_POSITION_X, self.turn_y))
                    pass

        # REVERSE PARKING LOGIC (P3) 
        elif self.final_gear == "R":
            is_case = ((abs(target_center_x - 17.5) < 0.1 or abs(target_center_x - 21.5) < 0.1) and 
                              (self.target_line == 3 or self.target_line == 2))
            if is_case:
                # WP 1bAlign car to alley Y
                Y_ALIGNMENT_X = car_x + 1.4 
                self.waypoints.append((Y_ALIGNMENT_X, self.turn_y))
                # WP 2 Drive past slot center for angle
                PRE_POSITION_X = target_center_x + CAR_LENGTH * 1.4
                self.waypoints.append((PRE_POSITION_X, self.turn_y)) 

            else:
                # WP 1 Align car to alley Y
                Y_ALIGNMENT_X = car_x + 1.0 
                self.waypoints.append((Y_ALIGNMENT_X, self.turn_y)) 
                # WP 2 Drive past slot center for angle
                PRE_POSITION_X = target_center_x + CAR_LENGTH * 1.2
                self.waypoints.append((PRE_POSITION_X, self.turn_y))
                
            # WP 3 Pivot Point
            # Waypoints finished
            #Gear shift to "park" happens after the last WP
            pass 
        
        self.navigation_phase = "go_to_waypoint"
        # Debug -> Display created waypoints
        print(f"[algo] Waypoints created : {self.waypoints}")

    def compute_control(self, obs: Dict[str, Any]) -> Dict[str, float]:
        """경로를 따라가기 위한 조향/가감속 명령을 산출합니다."""
        if self.navigation_phase == "init":
            self.compute_path(obs)
        
        if self.debug_counter == 0:
            print("[algo] compute_control called - starting navigation")
        
        # Default gear is 'D' for the 'go_to_waypoint' phase
        cmd = {"steer": 0.0, "accel": 0.0, "brake": 0.0, "gear": "D"}
        state = obs.get("state", {})
        # Get car current state (position, yaw, velocity)
        car_x = float(state.get("x", 0.0))
        car_y = float(state.get("y", 0.0))
        car_yaw = float(state.get("yaw", 0.0))
        car_v = float(state.get("v", 0.0))

        target_slot = obs.get("target_slot")
        if not target_slot or len(target_slot) < 4:
            # If no valid target : try to move slightly or stop
            if abs(car_v) < 0.05:
                cmd["accel"] = 0.5         # Small nudge if stopped
            else:
                cmd["brake"] = 0.5         # Brake if moving
            return cmd

        # Calculate target slot center
        slot_xmin, slot_xmax, slot_ymin, slot_ymax = map(float, target_slot)
        slot_center_x = (slot_xmin + slot_xmax) / 2.0
        slot_center_y = (slot_ymin + slot_ymax) / 2.0
        
        # Obstacle Detection
        # Retrieve the list of walls from the instance
        walls = self.walls_rects 
        # Check for obstacle presence (slots, occupied areas, walls)
        has_left_obstacle = check_side_collision(car_x, car_y, car_yaw, 'left', 
                                                 self.slots, self.occupied_idx, target_slot, walls)
        has_right_obstacle = check_side_collision(car_x, car_y, car_yaw, 'right',
                                                  self.slots, self.occupied_idx, target_slot, walls)
        has_front_obstacle = check_side_collision(car_x, car_y, car_yaw, 'front',
                                                  self.slots, self.occupied_idx, target_slot, walls)
        has_front_left_obstacle = check_side_collision(car_x, car_y, car_yaw, 'front_left', 
                                                       self.slots, self.occupied_idx, target_slot, walls)
        has_front_right_obstacle = check_side_collision(car_x, car_y, car_yaw, 'front_right', 
                                                        self.slots, self.occupied_idx, target_slot, walls)
        has_rear_obstacle = check_side_collision(car_x, car_y, car_yaw, 'rear', 
                                                       self.slots, self.occupied_idx, target_slot, walls)                                                        
        has_rear_left_obstacle = check_side_collision(car_x, car_y, car_yaw, 'rear_left', 
                                                      self.slots, self.occupied_idx, target_slot, walls)
        has_rear_right_obstacle = check_side_collision(car_x, car_y, car_yaw, 'rear_right',
                                                       self.slots, self.occupied_idx, target_slot, walls)
        
        # Collision counters to prevent false alerts
        if has_left_obstacle:
            self.left_collision_count += 1
        else:
            self.left_collision_count = max(0, self.left_collision_count - 1)
        
        if has_right_obstacle:
            self.right_collision_count += 1
        else:
            self.right_collision_count = max(0, self.right_collision_count - 1)
        
        if has_front_obstacle:
            self.front_collision_count += 1
        else:
            self.front_collision_count = max(0, self.front_collision_count - 1)
        if has_front_left_obstacle: 
            self.front_left_collision_count += 1
        else:
            self.front_left_collision_count = max(0, self.front_left_collision_count - 1)
            
        if has_front_right_obstacle: 
            self.front_right_collision_count += 1
        else:
            self.front_right_collision_count = max(0, self.front_right_collision_count - 1)

        if has_rear_obstacle:
            self.rear_collision_count += 1
        else:
            self.rear_collision_count = max(0, self.rear_collision_count - 1)    

        if has_rear_left_obstacle: 
            self.rear_left_collision_count += 1
        else:
            self.rear_left_collision_count = max(0, self.rear_left_collision_count - 1)
            
        if has_rear_right_obstacle: 
            self.rear_right_collision_count += 1
        else:
            self.rear_right_collision_count = max(0, self.rear_right_collision_count - 1)

        # Obstacle confirmed after consecutive detections
        left_danger = self.left_collision_count >= 3
        right_danger = self.right_collision_count >= 3
        front_danger = self.front_collision_count >= 3
        front_left_danger = self.front_left_collision_count >= 3 
        front_right_danger = self.front_right_collision_count >= 3 
        rear_danger = self.rear_collision_count >=1
        rear_left_danger = self.rear_left_collision_count >= 1 
        rear_right_danger = self.rear_right_collision_count >= 1 

        # Determine the effective gear for danger detection
        # Gear is 'D' in 'go_to_waypoint' 
        #uses 'final_gear' in 'park' phase
        effective_gear = self.final_gear if self.navigation_phase == "park" else "D"

        # Ignore initial danger detection during startup (debug counter < 1)
        if self.debug_counter < 1 :
            left_danger = right_danger = front_left_danger = front_right_danger = False
            rear_left_danger = rear_right_danger = rear_danger = False

        # Waypoints managment
        if self.navigation_phase == "go_to_waypoint":
            if not self.waypoints:
                print("[algo] ERROR: No waypoints, switching to PARK")
                self.navigation_phase = "park"
                return cmd

            # Get coordinates and calculate distance to the next waypoint
            target_wp_x, target_wp_y = self.waypoints[0]
            dx = target_wp_x - car_x
            dy = target_wp_y - car_y
            distance = math.sqrt(dx**2 + dy**2)
            
            # 1. Check if the waypoint is reached
            WAYPOINT_THRESHOLD = 1.0
            if len(self.waypoints) == 1:
                # Use a tighter threshold for final alley waypoint
                 WAYPOINT_THRESHOLD = 0.5 
            
            if distance < WAYPOINT_THRESHOLD:
                self.waypoints.pop(0)
                print(f"[algo] Reached waypoint, {len(self.waypoints)} remaining.")
                
                if not self.waypoints:
                    # All approach waypoints are completed
                    #Start PARK phase
                    self.navigation_phase = "park"
                    print("[algo] All approach waypoints reached, starting PARK phase.")
                    return self.compute_control(obs) 
                else:
                    # Update target to the next waypoint
                    target_wp_x, target_wp_y = self.waypoints[0] 

            # 2. Steering control towards the waypoint
            angle_to_target = math.atan2(dy, dx)
            angle_error = angle_to_target - car_yaw
            # Normalize angle error between -pi and pi
            angle_error = ((angle_error + math.pi) % (2 * math.pi)) - math.pi
            
            steer_gain = 2.0
            cmd["steer"] = max(-0.7, min(0.7, angle_error * steer_gain))
            
            # 3. Speed control
            if len(self.waypoints) <= 2:
                target_speed = 2.5 #1.5
            else:
                target_speed = 6.0 #2.5
            
            # Slow down as approach distance decreases
            if distance < 5.0 and distance > WAYPOINT_THRESHOLD:
                target_speed = max(0.5, target_speed * (distance / 5.0))
            
            # Accelerate or brake to reach target speed
            if abs(car_v) < target_speed * 0.8:
                cmd["accel"] = 0.95
            else:
                cmd["brake"] = 0.1
                
        # Obstacle avoidance steering adjustments 
        #(side and front corners)
        if left_danger or front_left_danger:
            cmd["steer"] -= 0.35             # Shift steering right (away from left obstacle)
        elif right_danger or front_right_danger:
            cmd["steer"] += 0.35 
        
        # Final parking maneuver
        elif self.navigation_phase == "park":
            # Calculate distance and angle error to slot center
            dx = slot_center_x - car_x
            dy = slot_center_y - car_y
            distance = math.sqrt(dx**2 + dy**2)
            
            angle_to_target = math.atan2(dy, dx)

            if self.final_gear == "R":
                angle_to_target += math.pi

            angle_error = angle_to_target - car_yaw
            angle_error = ((angle_error + math.pi) % (2 * math.pi)) - math.pi
            
            steer_gain = 5.0#2.5
            # Reverse steering direction if backing up ('R' gear)
            steer_multiplier = 1.0 if self.final_gear == "D" else -1.0

            cmd["steer"] = max(-0.5, min(0.5, angle_error * steer_gain * steer_multiplier)) 
            cmd["gear"] = self.final_gear

            # Speed control for parking 
            if distance > 1.5:
                target_speed = 0.9
                if abs(car_v) < target_speed:
                    cmd["accel"] = 0.3
                else:
                    cmd["brake"] = 0.2
            elif distance > 0.3:
                target_speed = 0.5
                if abs(car_v) < target_speed:
                    cmd["accel"] = 0.2
                else:
                    cmd["brake"] = 0.3
            else:
                # Hard brake near the final spot
                cmd["accel"] = 0.0
                cmd["brake"] = 0.8

        # Final safety check (ensures car starts if stationary and not braking hard)
        if abs(car_v) < 0.1 and cmd["accel"] < 0.3 and cmd["brake"] < 0.5:
            cmd["accel"] = 1.0
            cmd["brake"] = 0.0

        self.debug_counter += 1
        if self.debug_counter <= 3 or self.debug_counter % 30 == 0:
            obstacles_info = ""
            # Only display if frontal/side danger exists
            if front_danger:
                obstacles_info += " [FRONT]"
            if left_danger or front_left_danger:
                obstacles_info += " [LEFT/FL]"
            if right_danger or front_right_danger:
                obstacles_info += " [RIGHT/FR]"
            
            # Add current gear to the debug log
            current_gear = cmd.get("gear", "D")
            print(f"[algo] step={self.debug_counter} phase={self.navigation_phase} gear={current_gear} "
                  f"pos=({car_x:.1f},{car_y:.1f}) yaw={math.degrees(car_yaw):.0f}° v={car_v:.2f} ")

        return cmd

# 전역 planner 인스턴스 (통신 모듈이 이 객체를 사용합니다.)
planner = PlannerSkeleton()


def handle_map_payload(map_payload: Dict[str, Any]) -> None:
    """통신 모듈에서 맵 패킷을 받을 때 호출됩니다."""

    planner.set_map(map_payload)


def planner_step(obs: Dict[str, Any]) -> Dict[str, Any]:
    """통신 모듈에서 매 스텝 호출하여 명령을 생성합니다."""
    try:
        return planner.compute_control(obs)
    except Exception as exc:
        # Handle exceptions gracefully
        import traceback
        print(f"[algo] planner_step error: {exc}")
        traceback.print_exc()
        return {"steer": 0.0, "accel": 0.0, "brake": 0.5, "gear": "D"}