# Quick Reference Guide: Motor Controller Classes

## Table of Contents
1. [Controller Initialization](#controller-initialization)
2. [Using GrappleController](#using-grapplecontroller)
3. [Using AVCController](#using-avccontroller)
4. [Common Patterns](#common-patterns)
5. [Error Handling](#error-handling)

## Controller Initialization

### In GRASPNode.__init__():

```python
# Helper function to extract motor parameters from ROS params
def get_motor_params(motor_name: str) -> dict:
    idx = self.get_parameter('solo_params.name').get_parameter_value().string_array_value.index(motor_name)
    return {
        'address': self.get_parameter('solo_params.address').get_parameter_value().integer_array_value[idx],
        'baudrate': self.get_parameter('solo_params.baudrate').get_parameter_value().integer_array_value[idx],
        # ... all other parameters
    }

# Create controllers
grapple_params = get_motor_params('grapple')
self.grapple_controller = GrappleController(grapple_params, self.get_logger(), self.available_ports)

avc_a_params = get_motor_params('avc_a')
self.avc_a_controller = AVCController('avc_a', avc_a_params, self.get_logger(), self.available_ports)

avc_b_params = get_motor_params('avc_b')
self.avc_b_controller = AVCController('avc_b', avc_b_params, self.get_logger(), self.available_ports)
```

## Using GrappleController

### Available Methods

#### 1. Home the Grapple
```python
# Returns True when homing is complete, False if still in progress
is_homed = self.grapple_controller.home()

if is_homed:
    # Proceed to next state
    pass
```

#### 2. Move to Free Flight
```python
# Placeholder - not yet implemented
self.grapple_controller.free_flight()
```

#### 3. Open the Grapple
```python
# Returns True when fully open, False if still moving
is_open = self.grapple_controller.open()

if is_open:
    # Grapple is now at OPEN position (3900 QP)
    pass
```

#### 4. Soft Dock
```python
# Returns True when soft dock position is reached
is_soft_docked = self.grapple_controller.soft_dock()

if is_soft_docked:
    # Can proceed to hard dock
    pass
```

#### 5. Hard Dock
```python
# Returns True when hard dock force is achieved
is_hard_docked = self.grapple_controller.hard_dock()

if is_hard_docked:
    # Grapple is securely docked
    pass
```

#### 6. Move to Clearance (Release)
```python
# Returns True when clearance position is reached
is_clear = self.grapple_controller.clearance()

if is_clear:
    # Safe to separate
    pass
```

### State Checking

```python
# Get current state
current_state = self.grapple_controller.get_state()

if current_state == GrappleState.HOME:
    # Grapple is homed
    pass

# Check if homed
if self.grapple_controller.is_homed():
    # Mechanism has been homed at least once
    pass

# Check for errors
if self.grapple_controller.is_error():
    # Motor failed to initialize
    pass
```

### Reading Feedback

```python
# Must call update_feedback() first (usually at start of state machine)
self.grapple_controller.update_feedback()

# Access properties
position = self.grapple_controller.position      # Current position [QP]
speed = self.grapple_controller.speed            # Current speed [QPS]
current = self.grapple_controller.current        # Current [A]
pos_ref = self.grapple_controller.position_ref   # Position reference [QP]
speed_ref = self.grapple_controller.speed_ref    # Speed reference [QPS]
torque_ref = self.grapple_controller.torque_ref  # Torque reference [A]
```

## Using AVCController

### Available Methods

#### 1. Home the AVC
```python
# Returns True when homing is complete
is_homed = self.avc_a_controller.home()

if is_homed:
    # AVC is at HOME position
    pass
```

#### 2. Move to Launch Lock
```python
# Placeholder - not yet implemented
self.avc_a_controller.launch_lock()
```

#### 3. Move to Position 1
```python
# Returns True when position is reached
at_pos1 = self.avc_a_controller.pos1()

if at_pos1:
    # AVC is at POS1
    pass
```

#### 4. Move to Position 1.5
```python
# Returns True when position is reached
at_pos1p5 = self.avc_a_controller.pos1p5()
```

#### 5. Move to Position 2
```python
# Returns True when position is reached
at_pos2 = self.avc_a_controller.pos2()
```

### State Checking

```python
# Get current state
current_state = self.avc_a_controller.get_state()

if current_state == AVCState.HOME:
    # AVC is homed
    pass

# Same is_homed() and is_error() methods as GrappleController
```

### Reading Feedback

```python
# Same pattern as GrappleController
self.avc_a_controller.update_feedback()

position = self.avc_a_controller.position
speed = self.avc_a_controller.speed
current = self.avc_a_controller.current
```

## Common Patterns

### Pattern 1: State Machine Entry Actions

```python
case GRASPMode.SOME_MODE:
    """Mode description"""
    
    # Entry action: call controller method
    if not self.grapple_controller.is_error():
        operation_complete = self.grapple_controller.some_method()
        
        # Exit condition based on completion
        if operation_complete:
            self.GRASP_mode = GRASPMode.NEXT_MODE
            self.get_logger().info('Transitioned to NEXT_MODE')
```

### Pattern 2: Non-blocking State Transitions

```python
case GRASPMode.HOMING:
    # Call home() every state machine cycle until complete
    if not self.grapple_controller.is_error():
        homed = self.grapple_controller.home()
    
    # Check if all mechanisms are homed
    if (self.grapple_controller.get_state() == GrappleState.HOME and
        self.avc_a_controller.get_state() == AVCState.HOME and
        self.avc_b_controller.get_state() == AVCState.HOME):
        self.GRASP_mode = GRASPMode.READY
```

### Pattern 3: Update and Publish

```python
def GRASP_state_machine(self):
    # 1. Update all feedback at start
    self.grapple_controller.update_feedback()
    self.avc_a_controller.update_feedback()
    self.avc_b_controller.update_feedback()
    
    # 2. Run state machine logic
    match self.GRASP_mode:
        case GRASPMode.SOME_MODE:
            # ... state logic ...
            pass

def publish_data(self):
    # Use controller properties directly
    if not self.grapple_controller.is_error():
        pos_msg = Int32()
        pos_msg.data = self.grapple_controller.position
        self.pub_gra_motor_pos.publish(pos_msg)
```

## Error Handling

### Checking for Initialization Errors

```python
# During initialization
if self.grapple_controller.is_error():
    self.get_logger().error("Grapple controller failed to initialize")
    # Don't create publishers for this controller
else:
    # Create publishers
    self.pub_gra_motor_pos = self.create_publisher(Int32, 'gra_motor_pos', 10)
```

### Runtime Error Checking

```python
# In state machine
if not self.grapple_controller.is_error():
    # Safe to use controller
    self.grapple_controller.home()
else:
    # Skip this controller or handle error
    self.get_logger().warning("Skipping grapple operations - controller in error state")
```

### State-based Error Checking

```python
# Check if mechanism is in error state
grapple_state = self.grapple_controller.get_state()

if grapple_state == GrappleState.ERROR:
    # Handle error state
    self.GRASP_mode = GRASPMode.ERROR
```

## Advanced Usage

### Direct Motor Control (if needed)

The controllers expose control methods for special cases:

```python
# Position control
self.grapple_controller.position_control(target_position)

# Speed control
self.grapple_controller.speed_control(target_speed)  # negative = clockwise

# Torque control
self.grapple_controller.torque_control(target_torque)  # negative = clockwise

# Reset position counter
self.grapple_controller.reset_position()
```

### Accessing Configuration Constants

```python
# Read configuration from controller class
home_current = GrappleController.HOME_CURRENT_TARGET
open_position = GrappleController.OPEN_POSITION_TARGET

# These can be modified for testing (not recommended in production)
GrappleController.OPEN_POSITION_TARGET = 4000  # Temporary override
```

## Tips and Best Practices

1. **Always call `update_feedback()` at the start of your state machine cycle**
   - This ensures you have current motor data before making decisions

2. **Check `is_error()` before using a controller**
   - Prevents crashes if motor initialization failed

3. **Use the return values from movement methods**
   - They indicate when an operation is complete
   - Allows for non-blocking state transitions

4. **Don't mix direct motor control with controller methods**
   - Let the controller manage its own state
   - Only use direct control for debugging or special cases

5. **State checking is cheap**
   - `get_state()` is a simple property access
   - Use it freely for conditional logic

6. **Configuration constants should eventually move to params.yaml**
   - For now they're class constants
   - Makes tuning easier when in config file

## Debugging

### Print Current State
```python
self.get_logger().info(f"Grapple state: {self.grapple_controller.get_state().name}")
```

### Print Feedback Values
```python
self.get_logger().debug(
    f"Grapple - Pos: {self.grapple_controller.position}, "
    f"Speed: {self.grapple_controller.speed}, "
    f"Current: {self.grapple_controller.current}"
)
```

### Check Control Mode
```python
self.get_logger().debug(f"Control mode: {self.grapple_controller.control_mode}")
```

## Example: Complete Homing Sequence

```python
case GRASPMode.HOMING_SEQUENCE:
    """Home all mechanisms in sequence: AVC_A → AVC_B → Grapple"""
    
    # Stage 1: Home AVC A
    if self.avc_a_controller.get_state() != AVCState.HOME:
        if not self.avc_a_controller.is_error():
            self.avc_a_controller.home()
        return  # Wait for AVC A to complete
    
    # Stage 2: Home AVC B
    if self.avc_b_controller.get_state() != AVCState.HOME:
        if not self.avc_b_controller.is_error():
            self.avc_b_controller.home()
        return  # Wait for AVC B to complete
    
    # Stage 3: Home Grapple
    if self.grapple_controller.get_state() != GrappleState.HOME:
        if not self.grapple_controller.is_error():
            self.grapple_controller.home()
        return  # Wait for Grapple to complete
    
    # All homed - transition to next mode
    self.GRASP_mode = GRASPMode.READY
    self.get_logger().info("All mechanisms homed successfully")
```
