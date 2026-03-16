# Debug Guide: Communication Blackouts & RotateToObjectCommand Timeout

## Current Status

### Code Optimizations Implemented
✅ **DriveSubsystem**:
- Gyro rotation cached every 3 frames (↓ 67% I/O)
- Module positions cached every 5 frames (↓ 80% I/O)
- Odometry uses cached values

✅ **Limelight Camera**:
- NetworkTables reads cached every 3 frames (↓ 67% I/O)
- All getter methods return cached values
- Heartbeat monitoring with error handling

✅ **RotateToObjectCommand**:
- Reads camera values every 3 frames (already cached)
- Uses tolerance-based finish detection
- 5-second timeout as fallback

### Issues Remaining

#### Issue 1: Communication Blackouts During RotateToObjectCommand
**Possible Causes**:
1. **Limelight not connected**: Camera may not be sending data
2. **NetworkTables connectivity**: Possible network lag even with caching
3. **Odometry still blocking**: Module position reads happening during command execution

**How to Debug**:
1. Check if `[LIMELIGHT]` debug messages appear (currently commented out)
2. Monitor heartbeat status - should see "UPDATING" messages every 2 seconds
3. Look for any error messages in the console

#### Issue 2: RotateToObjectCommand Times Out Instead of Finishing
**Possible Root Causes**:

**A. April Tag Not Detected**:
- If `camera.getX()` always returns 0, and `x_offset = 0`
- Then `abs(0) < 3.0` is TRUE, so should finish... BUT
- `isFinished()` now requires `frame_counter > 5` for stability
- **Check console for**: `[ROTATE] Frame X: camera.getX() = ...` messages

**B. Limelight Not Connected to NetworkTables**:
- Camera might not be publishing data to NT
- Cache would remain at initial value (0.0)
- Command would see x_offset=0 and should finish... unless there's a logic error

**C. hasDetection() Returning False**:
- `hasDetection()` checks: `getX() != 0.0 AND heartbeating`
- If either is false, no detection

**What to Look For in Console Output**:
```
[ROTATE] Command initialized. Camera has_detection: <TRUE/FALSE>
[ROTATE] Frame 0: camera.getX() = <VALUE>, has_detection = <TRUE/FALSE>
[ROTATE] Frame 3: camera.getX() = <VALUE>, has_detection = <TRUE/FALSE>
...
[ROTATE] ALIGNED! x_offset=<VALUE>° (within 3.0°)
```

OR if timing out:
```
[ROTATE] TIMEOUT after 5.0s (x_offset=<VALUE>)
```

## Testing Procedure

### Step 1: Verify Limelight Connection
1. Enable the debug print in `limelight.py` by uncommenting the line:
   ```python
   print(f"[LIMELIGHT] Updated cache: tx={self.cached_tx:.2f}, ...")
   ```
2. Deploy code
3. **Expected**: See `[LIMELIGHT]` messages every ~60ms (cached every 3 frames)
4. **If not**: Limelight not connected or not publishing

### Step 2: Verify RotateToObjectCommand Starts
1. Activate RotateToObjectCommand via button binding
2. **Expected**: See `[ROTATE] Command initialized. Camera has_detection: TRUE`
3. **If FALSE**: Limelight not detecting April tag

### Step 3: Monitor During Rotation
1. Watch console for frame-by-frame debug output
2. **Expected**: See `[ROTATE] Frame X: camera.getX() = ...` every 3 frames
3. **Expected**: See offset decreasing as robot rotates (0 = centered)

### Step 4: Verify Finish Condition
1. **Expected to see one of**:
   - `[ROTATE] ALIGNED! x_offset=...` (command finishes normally)
   - `[ROTATE] TIMEOUT after 5.0s` (command times out)

## Communication Blackout Diagnosis

### If Blackouts Still Occur:

**Check 1: When Do They Occur?**
- Only during RotateToObjectCommand? → Likely Limelight blocking
- Randomly during normal drive? → Likely odometry or gyro blocking
- After button press? → Likely command initialization issue

**Check 2: Frequency**
- Every few seconds? → Might be periodic network read spiking
- Constant during command? → Likely continuous blocking I/O

**Check 3: Recovery Time**
- Instant (< 100ms)? → Brief hiccup, probably cache update
- Longer (> 500ms)? → Blocking operation in progress

### If Limelight is the Problem:

**Short-term solutions**:
1. Disable RotateToObjectCommand temporarily
2. Verify communication stable without it
3. If yes → Limelight is the culprit

**Long-term solutions**:
1. Increase NetworkTables read cache interval (currently 3 frames)
2. Reduce RotateToObjectCommand read frequency (currently 3 frames)
3. Use separate thread for camera updates (advanced)

### If Odometry is the Problem:

**Already optimized**:
- Module reads: every 5 frames
- Gyro reads: every 3 frames
- Using cached values for updates

**If still blocking**:
1. Increase cache intervals further (currently 5 for modules, 3 for gyro)
2. Disable odometry updates during autonomous (if applicable)
3. Consider separate CAN bus thread

## Quick Fixes to Try

### Fix 1: Increase Limelight Cache Interval
```python
# In limelight.py periodic():
if self.nt_read_counter >= 5:  # Changed from 3
    # read values
    self.nt_read_counter = 0
```

### Fix 2: Increase RotateToObjectCommand Update Frequency (for faster alignment)
```python
# In rotatetoobjectcommand.py execute():
if self.frame_counter % 5 == 0:  # Read less frequently
    self.last_x_offset = self.camera.getX()
```

### Fix 3: Disable RotateToObjectCommand Temporarily
Comment out the button binding in `robotcontainer.py` to test if that's causing blackouts

## Next Steps

1. **Deploy with debug prints enabled**
2. **Activate RotateToObjectCommand and observe console**
3. **Note what messages you see/don't see**
4. **Report findings** - this will help identify the exact bottleneck

## Key Debug Print Locations

| Location | Shows | When |
|----------|-------|------|
| `[LIMELIGHT]` | NT read cache updates | Every 3 frames (60ms) |
| `[ROTATE] Command initialized` | Command start, has_detection status | Button press |
| `[ROTATE] Frame X: camera.getX()` | Actual offset value from camera | Every 3 frames during command |
| `[ROTATE] ALIGNED!` | Command finishing normally | When centered |
| `[ROTATE] TIMEOUT` | Command giving up | After 5 seconds |

---
**Last Updated**: March 14, 2026
