import numpy as np
from math import degrees, sqrt, pow

# Constants
MAX_LIDAR_DISTANCE = 1.0  # Lidar maximum effective range
COLLISION_DISTANCE = 0.15  # Minimum safe distance to walls or obstacles
NEARBY_DISTANCE = 0.35  # Distance indicating proximity to walls
GOAL_NEAR_THRESHOLD = 0.3  # Goal proximity threshold

ZONE_0_LENGTH = 0.3
ZONE_1_LENGTH = 0.6

ANGLE_MAX = 360
ANGLE_MIN = 0
HORIZON_WIDTH = 60  # Angular range for forward and side sectors

# Discretize the Lidar scan into actionable zones
def scanDiscretization(state_space, lidar):
    """Discretize Lidar data into states."""
    # Initialize zones and sectors
    x1 = 2  # Left zone (default: no obstacle detected)
    x2 = 2  # Right zone (default: no obstacle detected)
    x3 = 3  # Left sector (default: no obstacle detected)
    x4 = 3  # Right sector (default: no obstacle detected)

    # Detect the closest obstacle on the left
    lidar_left = min(lidar[ANGLE_MIN:ANGLE_MIN + HORIZON_WIDTH])
    if ZONE_1_LENGTH > lidar_left > ZONE_0_LENGTH:
        x1 = 1  # Zone 1 (medium distance)
    elif lidar_left <= ZONE_0_LENGTH:
        x1 = 0  # Zone 0 (very close)

    # Detect the closest obstacle on the right
    lidar_right = min(lidar[ANGLE_MAX - HORIZON_WIDTH:ANGLE_MAX])
    if ZONE_1_LENGTH > lidar_right > ZONE_0_LENGTH:
        x2 = 1  # Zone 1 (medium distance)
    elif lidar_right <= ZONE_0_LENGTH:
        x2 = 0  # Zone 0 (very close)

    # Detection of obstacles in the front-left and front-right sectors
    object_front_left = min(lidar[ANGLE_MIN:HORIZON_WIDTH]) < NEARBY_DISTANCE
    object_front_right = min(lidar[ANGLE_MAX - HORIZON_WIDTH:ANGLE_MAX]) < NEARBY_DISTANCE

    # Adjust left and right sectors based on the detected obstacles
    if object_front_left:
        x3 = 0 if lidar_left < NEARBY_DISTANCE else 1
    if object_front_right:
        x4 = 0 if lidar_right < NEARBY_DISTANCE else 1

    # Find the state space index
    ss = np.where(np.all(state_space == np.array([x1, x2, x3, x4]), axis=1))
    state_ind = int(ss[0][0]) if ss[0].size > 0 else -1

    return state_ind, x1, x2, x3, x4

# Check if the robot crashed
def checkCrash(lidar):
    """Detect if the robot is too close to obstacles."""
    # Focus on the front half of the robot's field of view
    lidar_horizon = np.concatenate((
        lidar[ANGLE_MIN:HORIZON_WIDTH],
        lidar[ANGLE_MAX - HORIZON_WIDTH:ANGLE_MAX]
    ))
    # Weighted calculation to account for closer objects
    weights = np.linspace(1.2, 1.0, len(lidar_horizon) // 2)
    weights = np.concatenate((weights, weights[::-1]))
    return np.min(weights * lidar_horizon) < COLLISION_DISTANCE

# Check if the robot is near the goal
def checkGoalNear(x, y, x_goal, y_goal):
    """Check if the robot is close to the goal position."""
    distance = sqrt(pow((x_goal - x), 2) + pow((y_goal - y), 2))
    return distance < GOAL_NEAR_THRESHOLD

# Check if an object is nearby
def checkObjectNearby(lidar):
    """Detect if there are objects close to the robot."""
    lidar_horizon = np.concatenate((
        lidar[ANGLE_MIN:HORIZON_WIDTH],
        lidar[ANGLE_MAX - HORIZON_WIDTH:ANGLE_MAX]
    ))
    weights = np.linspace(1.3, 1.0, len(lidar_horizon) // 2)
    weights = np.concatenate((weights, weights[::-1]))
    return np.min(weights * lidar_horizon) < NEARBY_DISTANCE

