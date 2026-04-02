import math


PHYSICS_DT = 0.25               # Physics tick interval (seconds)
DISPLAY_LAG_SEC = 1.0           # Display lag behind physics (seconds)
DISPLAY_SMOOTHING = 0.28        # Velocity smoothing factor (0-1)
MAX_VISUAL_LEAD_M = 8.0         # Max forward visual lead (meters)


MAX_ACCEL = 2.0                 # Maximum acceleration (m/s^2)
MAX_DECEL = 2.9                 # Maximum deceleration (m/s^2)
MAX_TURN_RATE_RAD = math.radians(35.0)


SPEED_HOLD_SEC = 9.0            # Hold GPS speed duration (seconds)
VELOCITY_DAMPING = 0.88         # Velocity decay rate (per second)
BRAKE_STRENGTH = 1.0            # Brake bias during dead reckoning


DEAD_RECKON_START_SEC = 18.0    # Start dead reckoning after (seconds)
DEAD_RECKON_STOP_CONF = 0.2     # Force stop at confidence <= this
CONFIDENCE_DECAY_PER_SEC = 0.93 # Confidence decay rate
MAX_DEAD_RECKON_DIST = 100.0    # Maximum dead reckoning distance (meters)