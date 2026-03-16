# Limelight 4 Configuration for Best AprilTag Recognition

## Hardware Manager Settings

### Camera Settings
- **Resolution**: 1280x800 (higher resolution for better accuracy)
- **Frame Rate**: 60 FPS (faster detection)
- **Exposure**: 
  - For outdoor/bright lighting: Set to `Exposure Priority` mode
  - For controlled lighting: Auto exposure works well
  - If tags are washed out: Reduce exposure value to 1-5
  - If tags are too dark: Increase exposure value to 10-15
- **White Balance**: Auto (or manual if lighting is inconsistent)

### Pipeline Settings for AprilTag Detection
1. **Create a new pipeline specifically for AprilTags**
   - Input: USB Camera (if using direct camera) or Network (if using LL4's built-in)
   
2. **Select AprilTag Detector**
   - Family: Standard AprilTag family (for FRC tags)
   - Decide whether to detect 16h5 or similar family
   - Enable multi-tag detection if needed

3. **Detection Settings**
   - **Decimate**: 2-4 (lower = faster, less accurate on small tags)
   - **Pose Iteration Count**: 2 (balance between speed and accuracy)
   - **Blur**: Disabled (adds latency without much benefit)

4. **Output Settings**
   - TX (horizontal offset): ✅ Enable
   - TY (vertical offset): ✅ Enable  
   - TA (target area): ✅ Enable
   - Pipeline latency: Monitor but don't change

5. **LED Mode**
   - Set to `Current Pipeline` mode (respects your code settings)
   - Brightness: 100% for dark environments, 50% for bright

### Network Settings
- **Network Mode**: 
  - Set to **Microsoft Edge mode** (LL4 performs better on EdgeTPU)
  - Ensure 5GHz WiFi if available (lower interference)
  - Disable WiFi power saving if connection drops

### Connection Troubleshooting
- IP Address: Static recommended (192.168.1.11 or similar)
- Verify robot can ping limelight.local
- Check NetworkTables connection in Shuffleboard

## Code Optimization Notes

The following are critical for maintaining Driver Station communication:
- **Reduce Limelight reads**: Only read when needed, cache values
- **Batch NetworkTables operations**: Read multiple values in one operation if possible
- **Increase read interval**: 25Hz (every 2 frames at 50Hz) instead of 50Hz
- **Add timeout handling**: Gracefully handle when Limelight is unavailable

## Performance Targets
- **Latency**: Should see 20-35ms total (camera + processing + network)
- **FPS**: 50-60 FPS processing
- **Accuracy**: ±1-2 degrees at 10 feet away from AprilTag
