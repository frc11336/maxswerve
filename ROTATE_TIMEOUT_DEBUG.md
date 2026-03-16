# RotateToObjectCommand Timeout Debugging Guide

## Current Situation

- ✅ April tags ARE being detected (visible in Limelight hardware client)
- ✅ Communication blackouts are BETTER (but still occasional)
- ❌ RotateToObjectCommand still times out after 5 seconds
- ❓ Robot may not detect alignment, or x_offset values may not match expectations

## Diagnostic Output Added

The code now includes detailed debug output to help diagnose the issue.

### What You'll See When Command Executes

#### On Startup (once)
```
[ROTATE] ========== COMMAND STARTED ==========
[ROTATE] Alignment tolerance: 3.0°
[ROTATE] Rotation gain: 0.01
[ROTATE] Camera heartbeating: True
[ROTATE] Initial x_offset: 5.23
```

#### During Rotation (every 200ms / 10 frames)
```
[ROTATE] Frame 10: x_offset=4.85°, rotation_speed=0.0485, heartbeating=True
[ROTATE] Frame 20: x_offset=2.15°, rotation_speed=0.0215, heartbeating=True
[ROTATE] Frame 30: x_offset=0.45°, rotation_speed=0.0045, heartbeating=True
[ROTATE] Frame 40: x_offset=0.10°, rotation_speed=0.0010, heartbeating=True
```

#### On Success (when aligned)
```
[ROTATE] ========== ALIGNED at 0.10° ==========
```

#### On Timeout (after 5 seconds)
```
[ROTATE] ========== TIMEOUT after 5.0s ==========
[ROTATE] Final x_offset: 2.34°
[ROTATE] Final frame_counter: 250
[ROTATE] Alignment check: abs(2.34) < 3 = True
[ROTATE] Frame check: 250 > 5 = True
[ROTATE] Overall aligned: True
```

## Possible Issue #1: x_offset Never Goes Below 3°

**Symptom**: Debug output shows x_offset always > 3.0°

**Possible Causes**:
1. April tag too far to the side
2. Robot rotation speed too slow (offset decreases slowly)
3. Rotation direction wrong (robot rotating away instead of toward tag)

**How to Test**:
1. Position April tag DIRECTLY in front of robot
2. Activate command
3. Watch debug output - should see x_offset decrease
4. If it increases or stays high → rotation direction might be wrong

**Quick Fix if Rotation Direction is Wrong**:
```python
# In rotatetoobjectcommand.py, change this line:
rotation_speed = -self.ROTATION_GAIN * x_offset  # ← change sign here

# Try:
rotation_speed = self.ROTATION_GAIN * x_offset  # Remove the minus sign
```

## Possible Issue #2: x_offset Stabilizes Above 3°

**Symptom**: Debug output shows x_offset decreasing but stops at ~2-3° and doesn't improve

**Possible Causes**:
1. Rotation gain too low (0.01 might be too conservative)
2. Max rotation speed too low (0.5 might not provide enough torque)
3. Limelight calibration off

**How to Test**:
1. Check console output for: "rotation_speed" values
2. If rotation_speed is very small (< 0.01) → gain is too low
3. If rotation_speed hits limits (stays at 0.5) → max speed is too low

**Quick Fixes**:
```python
# Increase rotation gain (currently 0.01)
self.ROTATION_GAIN = 0.02  # Try doubling it

# Increase max rotation speed (currently 0.5)
self.MAX_ROTATION_SPEED = 0.75  # Try 75% instead of 50%
```

## Possible Issue #3: x_offset Zero But Command Doesn't Finish

**Symptom**: Debug output shows alignment check = True, but command still times out

**Possible Causes**:
1. `frame_counter > 5` check failing (shouldn't happen after 6 frames = 120ms)
2. Timing issue in isFinished() logic

**Diagnosis**:
Look at the timeout output - it should show:
```
[ROTATE] Frame check: 250 > 5 = True  ← This should be True
[ROTATE] Overall aligned: True
```

If "Overall aligned: True" but still times out → logic bug (unlikely)

## Possible Issue #4: Limelight Not Reading x Value Correctly

**Symptom**: All x_offset values are 0 or don't change

**How to Verify**:
1. Uncomment the debug print in Limelight.periodic():
   ```python
   print(f"[LIMELIGHT] Updated cache: tx={self.cached_tx:.2f}, ...")
   ```

2. Deploy and watch for:
   ```
   [LIMELIGHT] Updated cache: tx=5.23, ty=2.10, ta=1.50, hb=125
   [LIMELIGHT] Updated cache: tx=4.85, ty=2.05, ta=1.52, hb=126
   ```

3. If tx values don't change or stay at 0 → Limelight not publishing tx correctly

**How to Fix**:
1. Check Limelight web interface: `limelightyourname.local:5801`
2. Verify pipeline outputs `tx` (horizontal offset)
3. In hardware client, you should see the red crosshair on the April tag
4. The crosshair position = the tx value

## Step-by-Step Debugging Procedure

### Step 1: Verify Camera Is Working
```
Expected Console Output:
Camera limelight is UPDATING (hb=XXX)
```

If you see "NO LONGER UPDATING" → Limelight not connected to NetworkTables

### Step 2: Run Command and Watch Output
1. Position April tag 6-12 feet in front
2. Activate command
3. Copy the startup output
4. Paste in a text file for analysis

### Step 3: Analyze the Debug Output

**Questions to answer**:

1. Does x_offset decrease over time?
   - YES → Robot is rotating correctly
   - NO → Check rotation direction or max speed

2. Does x_offset reach below 3°?
   - YES → Good! Check frame_counter value
   - NO → Rotation is too slow or April tag too far

3. Is frame_counter > 5 before timeout?
   - YES → Should have finished (logic issue?)
   - NO → Something else wrong

4. Final x_offset value at timeout?
   - < 3° → Should have finished, but didn't (BUG)
   - > 3° → Not close enough, need more rotation

### Step 4: Report Findings

When you run the command, please note:
1. **Initial x_offset** value (from startup message)
2. **Final x_offset** value (from timeout message)
3. **Direction of change** (increasing or decreasing?)
4. **Minimum x_offset reached** (best alignment achieved)

## Quick Check Checklist

- [ ] April tags visible in Limelight hardware client?
- [ ] Red crosshair tracking the tag center?
- [ ] Heartbeat showing (hb counter increasing)?
- [ ] x_offset values visible in debug output?
- [ ] x_offset value decreasing when robot rotates?
- [ ] Robot actually rotating toward tag (audible/visible)?
- [ ] Camera settings configured for AprilTag detection?
- [ ] Limelight connected to robot NetworkTables?

## Common Solutions

### If x_offset never decreases
→ **Rotation direction is reversed**
```python
# Change in rotatetoobjectcommand.py:
rotation_speed = self.ROTATION_GAIN * x_offset  # Remove minus sign
```

### If x_offset decreases slowly
→ **Increase gain or max speed**
```python
self.ROTATION_GAIN = 0.02  # Double the gain
self.MAX_ROTATION_SPEED = 0.75  # Increase to 75%
```

### If x_offset = 0 at start (center aligned)
→ **Command should finish immediately** (check frame_counter requirement)

### If x_offset = 0 but command times out
→ **Logic bug - very unlikely, but check frame_counter > 5**

## NetworkTables Verification

If you want to manually verify what Limelight is sending:

**Option 1: OutlineViewer**
```
System > NetworkTables > Edit > Add Listener > limelight > tx
Watch the tx value change in real-time
```

**Option 2: Elastic Dashboard**
```
Add Number widget for: /limelight/tx
Watch the value as you rotate the April tag
```

## Next Steps

1. **Deploy code** with the new debug output
2. **Run RotateToObjectCommand** and collect the console output
3. **Copy/paste the full output** (from startup to timeout)
4. **Analyze** against the scenarios above
5. **Apply fix** if one of the scenarios matches
6. **Re-test** until command finishes successfully

---

## Example Successful Output

```
[ROTATE] ========== COMMAND STARTED ==========
[ROTATE] Alignment tolerance: 3.0°
[ROTATE] Rotation gain: 0.01
[ROTATE] Camera heartbeating: True
[ROTATE] Initial x_offset: 8.50
[ROTATE] Frame 10: x_offset=7.20°, rotation_speed=0.0720, heartbeating=True
[ROTATE] Frame 20: x_offset=5.50°, rotation_speed=0.0550, heartbeating=True
[ROTATE] Frame 30: x_offset=3.80°, rotation_speed=0.0380, heartbeating=True
[ROTATE] Frame 40: x_offset=2.10°, rotation_speed=0.0210, heartbeating=True
[ROTATE] Frame 50: x_offset=0.85°, rotation_speed=0.0085, heartbeating=True
[ROTATE] ========== ALIGNED at 0.85° ==========
```

This output shows:
- ✅ Camera working
- ✅ x_offset decreasing steadily
- ✅ Finished in ~1 second
- ✅ Command succeeded

---

**Last Updated**: March 14, 2026
**Status**: Ready for Diagnostic Testing
