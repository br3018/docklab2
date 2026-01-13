# GRASP Node Refactoring Summary

## Overview
The GRASP_node.py has been successfully refactored to use a class-based architecture for motor control. This improves code organization, readability, and maintainability.

## Architecture Changes

### New Class Hierarchy

```
MechanismController (Abstract Base Class)
├── GrappleController
└── AVCController
```

### 1. **MechanismController** (Base Class)
**Purpose:** Provides common functionality for all motor-driven mechanisms.

**Key Features:**
- Motor initialization and configuration
- Serial port detection and connection
- Control mode management (position, speed, torque)
- Feedback reading (position, speed, current)
- State management
- Error handling

**Methods:**
- `__init__()` - Initializes motor and sets up controller
- `_initialize_motor()` - Handles motor connection and configuration
- `update_feedback()` - Reads current motor feedback values
- `position_control()` - Commands position mode
- `speed_control()` - Commands speed mode
- `torque_control()` - Commands torque mode
- `reset_position()` - Resets encoder position to zero
- `get_state()` - Returns current mechanism state
- `is_homed()` - Checks if mechanism is homed
- `is_error()` - Checks if controller is in error state

**Properties:**
- `position` - Current motor position (QP)
- `speed` - Current motor speed (QPS)
- `current` - Current motor current (A)
- `position_ref` - Position reference value
- `speed_ref` - Speed reference value
- `torque_ref` - Torque reference value
- `control_mode` - Current control mode

### 2. **GrappleController** (Inherits from MechanismController)
**Purpose:** Specialized controller for the grapple mechanism.

**States Managed:**
- ERROR
- UNCONTROLLED
- HOME
- FREE_FLIGHT
- OPEN
- SOFT_DOCK
- HARD_DOCK
- CLEARANCE
- PAUSED

**Methods:**
- `home()` - Homes grapple using current control (-0.7A)
- `free_flight()` - Moves to free flight state (placeholder)
- `open()` - Opens grapple to 3900 QP
- `soft_dock()` - Soft docking using speed control (-2000 QPS)
- `hard_dock()` - Hard docking using current control (-0.7A)
- `clearance()` - Releases to clearance position (3600 QP)

**Configuration Constants:**
```python
HOME_CURRENT_TARGET = -0.7          # [A]
OPEN_POSITION_TARGET = 3900         # [QP]
SOFT_DOCK_POSITION_THRESHOLD = 3173 # [QP]
HARD_DOCK_CURRENT_TARGET = -0.7     # [A]
CLEARANCE_POSITION_TARGET = 3600    # [QP]
SOFT_DOCK_SPEED_TARGET = -2000      # [QPS]
```

### 3. **AVCController** (Inherits from MechanismController)
**Purpose:** Specialized controller for AVC (Active Vibration Control) mechanisms.

**States Managed:**
- ERROR
- UNCONTROLLED
- LAUNCH_LOCK
- HOME
- POS1
- POS1p5
- POS2

**Methods:**
- `home()` - Homes AVC using current control (0.08A)
- `launch_lock()` - Moves to launch lock state (placeholder)
- `pos1()` - Moves to position 1 (1000 QP - placeholder)
- `pos1p5()` - Moves to position 1.5 (1500 QP - placeholder)
- `pos2()` - Moves to position 2 (2000 QP - placeholder)

**Configuration Constants:**
```python
HOME_CURRENT_TARGET = 0.08  # [A]
POS1_TARGET = 1000          # [QP] - Needs configuration
POS1P5_TARGET = 1500        # [QP] - Needs configuration
POS2_TARGET = 2000          # [QP] - Needs configuration
```

## GRASPNode Changes

### Before (Old Implementation)
```python
# Direct motor control
self.grapple_Solo = self.motor_init('grapple')
self.grapple_state = GrappleState.UNCONTROLLED
self.grapple_homed = False

# Manual state management in state machine
if self.grapple_state != GrappleState.HOME:
    target_iq = -0.7
    if self.gra_motor_torque_ref != target_iq:
        self.motor_torque_control(self.grapple_Solo, target_iq)
    if abs(self.gra_motor_current) >= abs(target_iq):
        self.grapple_state = GrappleState.HOME
```

### After (New Implementation)
```python
# Controller-based approach
self.grapple_controller = GrappleController(grapple_params, self.get_logger(), self.available_ports)

# Simplified state machine
self.grapple_controller.home()
```

### Key Improvements

1. **Initialization:**
   - Old: Multiple try-except blocks with manual state tracking
   - New: Single controller instantiation with automatic error handling

2. **Feedback Reading:**
   - Old: Individual `get_*` calls for each motor in state machine
   - New: Single `update_feedback()` call per controller

3. **Publishing:**
   - Old: Access motor Solo object properties directly
   - New: Access controller properties (e.g., `controller.position`)

4. **State Machine:**
   - Old: Manual motor control with position/speed/torque checks
   - New: Call controller methods (e.g., `home()`, `open()`, `soft_dock()`)

## Benefits

### 1. **Improved Readability**
- State machine logic is much cleaner
- Intent is clearer (e.g., `grapple_controller.home()` vs manual current control)
- Less code duplication

### 2. **Better Encapsulation**
- Motor control logic is contained within controller classes
- State management is internal to controllers
- Node only needs to know about high-level operations

### 3. **Easier Maintenance**
- Motor control changes only need to be made in one place
- Configuration constants are clearly defined in controller classes
- Adding new mechanisms is straightforward (inherit from MechanismController)

### 4. **Improved Testability**
- Controllers can be unit tested independently
- Mock controllers can be created for testing state machine
- Clear interfaces between components

### 5. **Non-blocking Operation**
- Movement methods return completion status
- Allows periodic state machine updates
- No blocking waits

## Code Comparison

### Homing Grapple

**Before:**
```python
case GRASPMode.GRAPPLE_HOME:
    if self.grapple_state != GrappleState.ERROR:
        if self.grapple_state != GrappleState.HOME:
            target_iq = -0.7
            if self.gra_motor_torque_ref != target_iq or self.gra_motor_control_mode != solo.ControlMode.TORQUE_MODE:
                self.motor_torque_control(self.grapple_Solo, target_iq)
            
            if abs(self.gra_motor_current) >= abs(target_iq) and self.gra_motor_speed == 0:
                self.get_logger().debug('Current target for grapple homing reached.')
                self.motor_torque_control(self.grapple_Solo, 0)
                self.grapple_Solo.reset_position_to_zero()
                self.grapple_Solo.set_position_reference(0)
                self.grapple_state = GrappleState.HOME
                self.grapple_homed = True
```

**After:**
```python
case GRASPMode.GRAPPLE_HOME:
    if not self.grapple_controller.is_error():
        self.grapple_controller.home()
```

**Line reduction:** ~15 lines → 2 lines (87% reduction)

## Future Enhancements

### 1. **Configuration File Integration**
Move hardcoded constants to `params.yaml`:
```yaml
grapple:
  home_current_target: -0.7
  open_position_target: 3900
  soft_dock_position_threshold: 3173
  # etc...
```

### 2. **Actual AVC Position Values**
Replace placeholder values (1000, 1500, 2000 QP) with real values from testing.

### 3. **Implement Placeholders**
Complete the implementation of:
- `free_flight()` method in GrappleController
- `launch_lock()` method in AVCController
- AVC position movement modes in state machine (AVC_A_POS1, etc.)

### 4. **Add Position Tolerance Configuration**
Currently hardcoded at 10 QP in AVC controller, should be configurable.

### 5. **Enhanced Error Handling**
Add recovery mechanisms for error states.

### 6. **Logging Improvements**
Add more detailed logging levels and configurable verbosity.

## Migration Notes

### Breaking Changes
None - the external interface (ROS topics, services, commands) remains unchanged.

### Testing Recommendations
1. Test homing sequence for all three motors
2. Verify grapple open/close operations
3. Test soft dock and hard dock sequences
4. Check feedback publishing on all topics
5. Verify state transitions in response to commands

### Rollback Plan
The git history contains the previous implementation if rollback is needed.

## Files Modified
- `/home/labpi/docklab2_ws/src/docklab2/docklab2/GRASP_node.py` - Main node file with controller classes and refactored state machine

## Files Created
- `/home/labpi/docklab2_ws/src/docklab2/docklab2/mechanism_controllers.py` - Original separate controller file (now integrated into GRASP_node.py)
- `/home/labpi/docklab2_ws/src/docklab2/REFACTORING_SUMMARY.md` - This document

## Authors
- Original Implementation: [Original Author]
- Refactoring: GitHub Copilot & User
- Date: January 13, 2026
