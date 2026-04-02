# Motion Prediction Layer

A real-time GPS motion prediction system for vehicle tracking applications, designed to provide smooth, physics-based marker movement on maps between sparse GPS updates.

## 📋 Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Architecture](#architecture)
- [Use Cases](#use-cases)
- [Installation](#installation)
- [Integration Guide](#integration-guide)
- [API Reference](#api-reference)
- [Configuration](#configuration)
- [How It Works](#how-it-works)
- [Performance](#performance)


---

## Overview

The Motion Prediction Layer is a sophisticated prediction engine that transforms sparse, intermittent GPS data (typically arriving every 10-15 seconds) into smooth, continuous marker movement on digital maps. Built specifically for vehicle tracking applications, it eliminates the "jumping" effect of GPS updates and provides users with a natural, fluid tracking experience.

### Problem Statement

Traditional GPS tracking systems face a critical UX challenge:
- GPS updates arrive every 10-15 seconds (or longer during network issues)
- Direct GPS plotting causes markers to "teleport" between positions
- Users see jerky, unrealistic movement that breaks immersion
- No visual feedback during GPS blackouts (tunnels, signal loss)

### Solution

This prediction layer:
- ✅ Predicts smooth marker positions between GPS updates
- ✅ Uses physics-based motion modeling for realistic movement
- ✅ Follows route polylines to stay on roads during GPS uncertainty
- ✅ Implements magnetic wells for realistic stop behavior at designated locations
- ✅ Handles GPS blackouts gracefully with confidence decay
- ✅ Provides 60 FPS smooth rendering capability

---

## Key Features

### 🎯 Core Capabilities

#### 1. **Physics-Based Prediction**
- Velocity and acceleration modeling
- Speed hold after GPS updates (5 seconds)
- Gradual velocity damping (0.98/sec)
- Brake bias during dead reckoning
- Maximum acceleration/deceleration limits

#### 2. **Route-Following** 
- Marker always stays on visible route polyline
- Projects position onto route during all conditions
- Adapts instantly when backend reroutes
- Smooth convergence based on GPS confidence
- Handles route end gracefully

#### 3. **Magnetic Wells (Bus Stop Behavior)**
- **Awareness Zone** (150m-60m): Marker knows stop is approaching
- **Slowdown Zone** (60m-10m): Gradual deceleration aligned with GPS
- **Snap Zone** (<10m): Lock to exact stop position
- **Hold State**: Marker stays at stop until GPS confirms departure (>1.0 m/s)
- Uses **marker position** (what user sees) for activation
- Uses **GPS speed** (authoritative) for release

#### 4. **Display Smoothing**
- Lag compensation (0.5 second behind physics state)
- Velocity direction blending
- Acceleration limiting (1.0 m/s² for UI)
- No backward motion rule
- Visual lead cap (8 meters maximum)

#### 5. **GPS Uncertainty Handling**
- Confidence decay during GPS blackouts
- Teleport detection (bus-specific, max 100 km/h)
- Dead reckoning with brake bias (starts at 15 seconds)
- Hard stop at very low confidence (≤0.2)
- Smart recovery when GPS returns

---

## Architecture

### System Design

```
┌─────────────────────────────────────────────────────────────┐
│                    GPS DATA SOURCE                           │
│  (Device sends packets every 10-15s with lat/lon/speed)     │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                  ADAPTER (API Layer)                         │
│  • Parses GPS packets                                        │
│  • Converts units (km/h → m/s, ms → sec)                    │
│  • Manages route context                                     │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                  ENGINE (Vehicle Manager)                    │
│  • Manages multiple vehicles                                 │
│  • Routes GPS to correct predictor                          │
│  • Handles route updates                                     │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│               PREDICTOR (Per-Vehicle State)                  │
│  • Coordinates background and display states                 │
│  • Manages route geometry and stops                         │
│  • Runs physics ticks at 0.25s intervals                    │
└───────┬────────────────────────────────────┬────────────────┘
        │                                    │
        ▼                                    ▼
┌──────────────────────┐          ┌──────────────────────┐
│  BACKGROUND STATE    │          │   DISPLAY STATE      │
│  (Physics Engine)    │          │   (Visual Output)    │
│                      │          │                      │
│  • GPS integration   │          │  • Lag compensation  │
│  • Velocity damping  │          │  • Smoothing         │
│  • Route-following   │◄────────►│  • Magnetic wells    │
│  • Confidence decay  │          │  • Position output   │
│  • Marker tracking   │          │                      │
└──────────────────────┘          └──────────────────────┘
```

### Component Breakdown

#### **Adapter** (`adapter.py`)
- Public API for GPS ingestion and position queries
- Unit conversion and validation
- Route context management

#### **Engine** (`engine.py`)
- Multi-vehicle state management
- Predictor lifecycle management
- Route update propagation

#### **Predictor** (`predictor.py`)
- Per-vehicle prediction coordinator
- Physics tick execution (4 Hz)
- Background ↔ Display state synchronization

#### **Background State** (`state_background.py`)
- Authoritative physics simulation
- GPS fix application (STOP/MOVING/TELEPORT modes)
- Route-following when route available
- Marker position calculation
- Confidence decay

#### **Display State** (`state_display.py`)
- Lag-compensated output (0.5s behind)
- Velocity smoothing and acceleration limiting
- Magnetic well application
- Final position rendering

#### **Route Follower** (`route_follower.py`)
- Projects position onto route polyline
- Advances along route based on speed
- Handles segment transitions
- Smooth convergence to route

#### **Magnetic Well** (`magnetic_well.py`)
- Multi-phase stop approach behavior
- Distance-based activation (uses marker position)
- GPS-speed-based release (authoritative)
- Stop sequence tracking

#### **Route Geometry** (`route_geometry.py`)
- Route and stop data models
- Coordinate conversion (lat/lon ↔ XY)
- Direction handling (OUTBOUND/INBOUND)

---

## Use Cases

### Primary Use Case: City Bus Tracking

**Scenario**: Real-time bus tracking system for passengers

**Requirements**:
- Smooth marker movement on map UI
- Realistic stop behavior at bus stops
- Handle GPS gaps in tunnels/underground
- Stay on route during signal loss
- 60 FPS rendering capability

**Solution**: 
```python
# Set route once at journey start
adapter.update_route_context(
    device_id="BUS_001",
    route_polyline=[(lat1, lon1), (lat2, lon2), ...],
    stops=[("STOP_A", lat, lon), ("STOP_B", lat, lon), ...]
)

# Ingest GPS every 10-15 seconds
adapter.ingest_gps_packet(
    device_id="BUS_001",
    latitude="13.0352",
    longitude="77.5970",
    speed="40.0",
    timestamp=1234567890000
)

# Query at 60 FPS for smooth rendering
position = adapter.get_display_position("BUS_001")
# Returns: {"lat": 13.0352, "lon": 77.5970, "speed_kmph": 40.5, "confidence": 0.98}
```

### Secondary Use Cases

- **Fleet Management**: Real-time tracking of delivery vehicles
- **Ride-sharing**: Live driver location for passenger apps
- **Emergency Services**: Ambulance/police vehicle tracking
- **Logistics**: Shipment tracking with realistic ETA

---

## Installation

### Prerequisites

- Python 3.9+
- NumPy (for vector mathematics)
- Flask (for testing server, optional)

### Install from Source

```bash
# Clone the repository
git clone 

# Install dependencies
pip install -r requirements.txt

# Install the package
pip install -e .
```

### Project Structure

```
motion_prediction/
├── api/
│   └── adapter.py              # Public API
├── core/
│   ├── engine.py               # Vehicle manager
│   ├── predictor.py            # Per-vehicle predictor
│   ├── state_background.py     # Physics engine
│   ├── state_display.py        # Display smoothing
│   ├── route_follower.py       # Route-following logic
│   ├── route_geometry.py       # Route data models
│   └── magnetic_well.py        # Bus stop behavior
├── math/
│   ├── vector.py               # Vec2 implementation
│   └── kinematics.py           # Physics functions
├── utils/
│   └── geo.py                  # Coordinate conversion
├── models/
│   └── gps_packet.py           # GPS data model
└── config/
    └── settings.py             # Configuration constants
```

---






## Integration Guide

### Step 1: Install the Package

```bash
pip install 
```

### Step 2: Initialize Adapter

In your application startup:

```python
from motion_prediction.api.adapter import MotionPredictionAdapter

# Initialize once (singleton pattern recommended)
prediction_adapter = MotionPredictionAdapter(enabled=True)
```

### Step 3: Set Route Context

When a vehicle is assigned to a route:

```python
def on_vehicle_route_assigned(device_id, route_id, direction):
    """
    Called when admin assigns vehicle to route via UI
    
    Args:
        device_id: Vehicle device ID (e.g., "357803371749371")
        route_id: Route ID from your database
        direction: "OUTBOUND" or "INBOUND"
    """
    # Fetch route from your database
    route = get_route_with_coordinates(route_id)  # Your existing function
    
    # Extract coordinates
    route_polyline = [
        (float(coord['latitude']), float(coord['longitude']))
        for coord in route['routeCoordinates']
    ]
    
    # Extract stops (if available)
    stops = []
    if hasattr(route, 'stops'):
        stops = [
            (stop['stopId'], float(stop['latitude']), float(stop['longitude']))
            for stop in route['stops']
        ]
    
    # Reverse for INBOUND direction
    if direction == 'INBOUND':
        route_polyline = list(reversed(route_polyline))
        stops = list(reversed(stops))
    
    # Set route in prediction layer
    prediction_adapter.update_route_context(
        device_id=device_id,
        route_polyline=route_polyline,
        stops=stops
    )
    
    print(f"✅ Route set for {device_id}")
```

### Step 4: Ingest GPS Data

In your GPS processing pipeline:

```python
def process_gps_packet(gps_data):
    """
    Called whenever GPS packet arrives from device
    
    Args:
        gps_data: dict with device_id, latitude, longitude, speed, timestamp
    """
    # Your existing GPS processing
    save_to_database(gps_data)
    broadcast_to_websocket(gps_data)
    
    # NEW: Feed to prediction layer
    prediction_adapter.ingest_gps_packet(
        device_id=gps_data['device_id'],
        latitude=str(gps_data['latitude']),
        longitude=str(gps_data['longitude']),
        speed=str(gps_data['speed']),  # km/h
        timestamp=gps_data['timestamp']  # milliseconds
    )
```

### Step 5: Query Display Position

In your API endpoint that serves vehicle positions to frontend:

```python
from flask import Flask, jsonify
import time

app = Flask(__name__)

@app.route("/api/vehicles/<device_id>/position")
def get_vehicle_position(device_id):
    """
    API endpoint called by frontend (e.g., every 1 second or on animation frame)
    """
    # Get predicted position
    position = prediction_adapter.get_display_position(
        vehicle_id=device_id,
        now_ts=time.time()  # Optional, defaults to current time
    )
    
    if position is None:
        return jsonify({"error": "Vehicle not found"}), 404
    
    return jsonify({
        "device_id": device_id,
        "latitude": position['lat'],
        "longitude": position['lon'],
        "speed_kmph": position['speed_kmph'],
        "confidence": position['confidence'],
        "timestamp": position['timestamp']
    })
```

### Step 6: Handle Route Changes

When backend reroutes (Google Maps API, manual reroute, etc.):

```python
def on_route_updated(device_id, new_route_polyline):
    """
    Called when route changes (reroute, detour, etc.)
    """
    # Update prediction layer with new route
    prediction_adapter.update_route_context(
        device_id=device_id,
        route_polyline=new_route_polyline,
        stops=None  # Or new stops if changed
    )
    
    print(f"✅ Route updated for {device_id}")
```

### Step 7: Handle Journey End

When vehicle is removed from route:

```python
def on_journey_end(device_id):
    """
    Called when vehicle finishes journey
    """
    # Clear route from prediction layer
    prediction_adapter.update_route_context(
        device_id=device_id,
        route_polyline=None,
        stops=None
    )
    
    print(f"✅ Journey ended for {device_id}")
```

### Step 8: System Restart Recovery

On server startup, restore state for active vehicles:

```python
def on_server_startup():
    """
    Called when backend server starts
    """
    # Get all vehicles currently on journeys
    active_vehicles = get_active_vehicles_from_db()
    
    for vehicle in active_vehicles:
        if vehicle.route_id:
            route = get_route_with_coordinates(vehicle.route_id)
            
            route_polyline = [
                (float(c['latitude']), float(c['longitude']))
                for c in route['routeCoordinates']
            ]
            
            stops = [
                (s['stopId'], float(s['latitude']), float(s['longitude']))
                for s in route['stops']
            ]
            
            if vehicle.direction == 'INBOUND':
                route_polyline = list(reversed(route_polyline))
                stops = list(reversed(stops))
            
            prediction_adapter.update_route_context(
                device_id=vehicle.device_id,
                route_polyline=route_polyline,
                stops=stops
            )
    
    print(f"✅ Restored {len(active_vehicles)} active vehicles")
```

---

## API Reference

### `MotionPredictionAdapter`

#### Constructor

```python
MotionPredictionAdapter(enabled: bool = True)
```

**Parameters**:
- `enabled`: Enable/disable prediction layer

**Example**:
```python
adapter = MotionPredictionAdapter(enabled=True)
```

---

#### `ingest_gps_packet()`

```python
ingest_gps_packet(
    device_id: str,
    latitude: str,
    longitude: str,
    timestamp: int,
    speed: str | None = None
) -> None
```

**Purpose**: Ingest GPS data from vehicle

**Parameters**:
- `device_id`: Vehicle identifier (e.g., "357803371749371")
- `latitude`: GPS latitude as string (e.g., "13.0352")
- `longitude`: GPS longitude as string (e.g., "77.5970")
- `timestamp`: GPS timestamp in **milliseconds** (e.g., 1770284907126)
- `speed`: GPS speed in **km/h** as string (e.g., "40.0"), optional

**Returns**: None

**Example**:
```python
adapter.ingest_gps_packet(
    device_id="BUS_001",
    latitude="13.035200",
    longitude="77.597000",
    timestamp=1770284907126,
    speed="40.0"
)
```

---

#### `update_route_context()`

```python
update_route_context(
    device_id: str,
    route_polyline: List[Tuple[float, float]] | None,
    stops: List[Tuple[str, float, float]] | None = None
) -> None
```

**Purpose**: Set or update route and stops for a vehicle

**Parameters**:
- `device_id`: Vehicle identifier
- `route_polyline`: List of (latitude, longitude) tuples defining route path, or None to clear
- `stops`: List of (stop_id, latitude, longitude) tuples for bus stops, optional

**Returns**: None

**Example**:
```python
# Set route
adapter.update_route_context(
    device_id="BUS_001",
    route_polyline=[
        (13.0352, 77.5970),
        (13.0360, 77.5977),
        (13.0375, 77.5988)
    ],
    stops=[
        ("STOP_A", 13.0352, 77.5970),
        ("STOP_B", 13.0375, 77.5988)
    ]
)

# Clear route
adapter.update_route_context(
    device_id="BUS_001",
    route_polyline=None,
    stops=None
)
```

---

#### `get_display_position()`

```python
get_display_position(
    vehicle_id: str,
    now_ts: float | None = None
) -> dict | None
```

**Purpose**: Get current predicted marker position for display

**Parameters**:
- `vehicle_id`: Vehicle identifier
- `now_ts`: Current timestamp in **seconds** (optional, defaults to `time.time()`)

**Returns**: Dict with position data, or None if vehicle not found

**Response Format**:
```python
{
    "lat": float,          # Predicted latitude
    "lon": float,          # Predicted longitude
    "speed_kmph": float,   # Current speed in km/h
    "confidence": float,   # GPS confidence (0.0 to 1.0)
    "timestamp": float     # Query timestamp
}
```

**Example**:
```python
import time

position = adapter.get_display_position("BUS_001", time.time())

if position:
    print(f"Position: ({position['lat']}, {position['lon']})")
    print(f"Speed: {position['speed_kmph']} km/h")
    print(f"Confidence: {position['confidence']:.2%}")
```

---

#### `enable()` / `disable()`

```python
enable() -> None
disable() -> None
```

**Purpose**: Enable or disable prediction layer globally

**Example**:
```python
adapter.disable()  # Stop all predictions
adapter.enable()   # Resume predictions
```

---

## Configuration

### Physics Settings (`config/settings.py`)

```python
# Timing
PHYSICS_DT = 0.25              # Physics tick interval (seconds)
DISPLAY_LAG_SEC = 0.5          # Display lag compensation (seconds)

# Motion Limits
MAX_ACCEL = 2.5                # Maximum acceleration (m/s²)
MAX_DECEL = 4.0                # Maximum deceleration (m/s²)

# Velocity Behavior
SPEED_HOLD_SEC = 5.0           # Hold GPS speed duration (seconds)
VELOCITY_DAMPING = 0.98        # Velocity decay rate (per second)
BRAKE_STRENGTH = 2.2           # Brake bias strength during dead reckoning

# Confidence & Dead Reckoning
DEAD_RECKON_START_SEC = 15.0   # Start dead reckoning after (seconds)
DEAD_RECKON_STOP_CONF = 0.2    # Force stop at confidence ≤ this
CONFIDENCE_DECAY_PER_SEC = 0.961  # Confidence decay rate

# Display Behavior
DISPLAY_SMOOTHING = 0.2        # Velocity smoothing factor (0-1)
MAX_VISUAL_LEAD_M = 8.0        # Max display lead distance (meters)
```

### Magnetic Well Settings (`core/magnetic_well.py`)

```python
# Distance Thresholds
awareness_radius = 150.0       # Awareness zone (meters)
slowdown_radius = 60.0         # Slowdown zone (meters)
snap_radius = 10.0             # Snap zone (meters)

# Speed Thresholds
stopped_threshold = 0.5        # GPS speed to confirm stopped (m/s)
departure_threshold = 1.0      # GPS speed to confirm departure (m/s)
```

### Tuning Recommendations

**For Buses**:
- `SPEED_HOLD_SEC = 5.0` - Good for stop-start patterns
- `BRAKE_STRENGTH = 2.2` - Moderate braking during uncertainty
- `slowdown_radius = 60.0` - Comfortable deceleration distance

**For Cars/Taxis**:
- `SPEED_HOLD_SEC = 3.0` - Faster response
- `BRAKE_STRENGTH = 1.5` - Less aggressive braking
- `slowdown_radius = 40.0` - Shorter deceleration

**For Trucks**:
- `SPEED_HOLD_SEC = 7.0` - Account for momentum
- `BRAKE_STRENGTH = 3.5` - Stronger braking
- `MAX_ACCEL = 1.5` - Lower acceleration limit

---








## How It Works

### GPS Fix Application

When GPS packet arrives:

1. **Validate** coordinates and speed
2. **Detect mode**:
   - Speed < 0.3 m/s → **STOP**
   - Position jump > max reasonable → **TELEPORT**
   - Otherwise → **MOVING**
3. **Infer direction** from position delta
4. **Update state** with GPS position and velocity
5. **Reset confidence** to 1.0

### Physics Propagation (Every 0.25s)

1. **Integrate velocity** from acceleration
2. **Apply speed hold** if within 5s of GPS
3. **Apply damping** after speed hold period
4. **Apply brake bias** if confidence < 0.55
5. **Route-following** (if route available):
   - Project position onto route
   - Advance along route polyline
   - Update velocity to follow route direction
6. **Calculate marker position** (smoothed tracking)
7. **Decay confidence** based on time since GPS

### Display Position Query

1. **Run physics ticks** up to query time
2. **Sample history** with 0.5s lag
3. **Interpolate** between samples
4. **Smooth velocity** direction
5. **Apply acceleration limit** (1.0 m/s²)
6. **Apply GPS speed cap** (prevent overshoot)
7. **Apply magnetic well** (if near stop):
   - Check marker position distance to stops
   - Apply slowdown/snap/hold based on phase
   - Use GPS speed for release decision
8. **Return final position**

### Route-Following Details

```
┌────────────┐
│ GPS Update │
└──────┬─────┘
       │
       ▼
┌─────────────────┐
│ Current Position│
└──────┬──────────┘
       │
       ▼
┌──────────────────────┐
│ Locate on Route      │  ← Find closest point on polyline
└──────┬───────────────┘
       │
       ▼
┌──────────────────────┐
│ Advance by Distance  │  ← speed * dt along route
└──────┬───────────────┘
       │
       ▼
┌──────────────────────┐
│ Blend to Route       │  ← confidence-based (30%-100%)
└──────┬───────────────┘
       │
       ▼
┌──────────────────────┐
│ Update Velocity      │  ← Follow route direction
└──────────────────────┘
```

### Magnetic Well State Machine

```
AWARE (150m-60m)
    │
    ├─> Monitor approach
    │
    ▼
SLOWING (60m-10m)
    │
    ├─> Gradual deceleration
    ├─> Position pull (20% max)
    │
    ▼
SNAPPED (<10m)
    │
    ├─> Lock to exact stop position
    ├─> Wait for GPS speed < 0.5 m/s
    │
    ▼
STOPPED
    │
    ├─> Hold at stop
    ├─> Zero velocity
    ├─> Wait for GPS speed > 1.0 m/s
    │
    ▼
RELEASED
```

---







### Development Setup

```bash
# Clone repository
git clone 

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install in development mode
pip install -e ".[dev]"

# Run tests
pytest

# Run linter
flake8 motion_prediction/

# Format code
black motion_prediction/
```

---



