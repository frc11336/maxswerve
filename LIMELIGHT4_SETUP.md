# Limelight 4 Hardware Configuration for AprilTag Detection

## Web Interface Setup (limelightyourname.local:5801)

### Vision Tab → Camera Settings

#### Resolution & FPS
- **Resolution**: 640x480 (good balance of accuracy vs speed)
- **FPS**: 60 FPS (standard, Limelight caches every 3 frames → 20Hz effective)

#### Exposure Settings
- **Exposure Mode**: Manual (NOT Auto)
- **Exposure Time**: 10-15 ms (adjust for arena lighting)
  - **Darker area**: 15-20 ms
  - **Brighter area**: 5-10 ms
- **Gain**: 0-15 (keep low to reduce noise)

#### White Balance
- **Mode**: Manual (if available)
- **Kelvin**: 3500-5500K (typical indoor arena)

#### Focus
- **Mode**: Manual Fixed Focus (calibrate once)
- **Distance**: Set appropriate for your target distance from robot

### Vision Tab → Pipeline Settings

#### General
- **Pipeline Name**: AprilTag (or descriptive name)
- **Input Source**: RGB Camera

#### Detection Settings
- **Target Family**: 36h11 (FRC standard) + any others you use
- **Edge Refinement**: Enabled (improves accuracy)
- **Decoding Decimation**: 2-4 (balance of speed/accuracy)
  - Lower (1-2): More accurate, slower
  - Higher (3-4): Faster, less accurate

#### Region of Interest (ROI)
- **Full Frame** initially (can optimize later)
- Can crop edges if only front-facing tags

### Vision Tab → Output Settings

#### Critical Settings
- **Enable Targeting Data**: YES
- **Enable Raw Contours**: NO (reduces processing)
- **Enable JSON Dumps**: NO (reduces processing)

#### NetworkTables
- **Update Rate**: High (should be default)
- **NT Protocol**: NT4 (confirmed in your code)

### Vision Tab → LED Settings

- **LED Mode**: Enabled (ON)
- **Brightness**: 
  - **Arena with good lighting**: 30-50%
  - **Dim arena**: 70-100%
  - **Testing/Tuning**: 100%
- **Blink Pattern**: Solid (not blinking)

### Hardware Tab → Mounting/Calibration

#### Camera Calibration
1. Go to **Camera Calibration** section
2. Print AprilTag sheet (6 tags minimum)
3. Capture 20+ images at various angles/distances
4. Click "Calibrate"
5. Wait for calibration to complete

#### Lens Distortion
- Should be calculated by calibration
- Can manually adjust if calibration fails

## Code Configuration Verification

### Check These are Enabled in Your Code:

```python
# In robotcontainer.py - RotateToObjectCommand should be bound
self.driverController.x().onTrue(RotateToObjectCommand(self.robotDrive, self.camera))

# In subsystems/limelight.py - Should use NT4
# Verified: using NetworkTableInstance.getDefault() with NT4
```

## Troubleshooting Checklist

### April Tag Not Detected
- [ ] Limelight has power (green LED visible)
- [ ] Network connection (ethernet cable connected)
- [ ] Pipeline set to AprilTag detection
- [ ] LED enabled and bright enough
- [ ] April tags in good condition (not faded)
- [ ] Tags at reasonable distance (6-20 feet for FRC)
- [ ] Camera focused (manual focus calibrated)
- [ ] White balance matches arena lighting
- [ ] Exposure not too dark or too bright

### Limelight Web Interface Unreachable
- [ ] Check IP camera network connectivity
- [ ] Restart Limelight (power cycle)
- [ ] Verify firewall not blocking (should be on same subnet as robot)
- [ ] Try direct IP: `192.168.1.XX` (check router for assigned IP)

### Values Always Zero
- [ ] Pipeline not selected correctly
- [ ] April tags not in view of camera
- [ ] Limelight not connected to Robot Rio network
- [ ] NetworkTables not initialized

### Values Jumping Around (Noisy)
- [ ] Increase exposure time (more light)
- [ ] Increase LED brightness
- [ ] Reduce gain
- [ ] Improve lighting in arena
- [ ] Clean lens

## Network Diagram

```
Limelight 4 (Network Camera)
        ↓ (ethernet)
        ↓ (NT4 Publishing)
     Robot Rio
        ↓ (reads from NT)
    DriveSubsystem
        ↓ (requests)
   RotateToObjectCommand
        ↓ (caches every 3 frames)
    LimelightCamera subsystem
        ↓ (reads getX, getY, etc)
   RotateToObjectCommand.execute()
```

## Performance Targets

With current code optimizations:
- **Limelight NT reads**: ~17/second (every 3 frames)
- **RotateToObjectCommand reads**: ~17/second (from cache, every 3 frames)
- **Total blocking I/O**: Minimal
- **Expected response time**: 60-100ms (one cache cycle + processing)

## Important Notes

1. **Camera Calibration is Critical** - Without proper calibration, April tag detection accuracy suffers significantly
2. **Lighting is Critical** - Good, consistent lighting improves detection by 10x
3. **Lens Focus** - AprilTags must be in focus. Calibrate focus distance once and don't change
4. **LED Brightness** - Should illuminate tags evenly without washout

## Testing Procedure

1. **Enable RotateToObjectCommand** in button bindings
2. **Place AprilTag** in front of robot (6-15 feet away)
3. **Press button** to activate
4. **Watch console** for `[ROTATE]` debug messages
5. **Verify**:
   - ✓ `has_detection: TRUE` when tag visible
   - ✓ `camera.getX()` shows offset value (not 0)
   - ✓ Robot rotates toward tag
   - ✓ Command finishes when aligned

---
**Last Updated**: March 14, 2026
