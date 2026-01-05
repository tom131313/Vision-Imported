# FRC AprilTag Vision Examples

Quick start for FRC teams learning AprilTag detection and robot pose estimation. Detects AprilTags, computes robot position, and drives to game targets.

## Architecture

`VisionContainer` wraps three vision system implementations:
- `ControllerVision` (WPILib) - Free, requires only WPILib
- `PhotonVision` - Free, coprocessor recommended
- `LimelightVision` - Commercial hardware

## Example Commands

Three targeting strategies are implemented:

### 1. Turn and Drive to Distance
Turn to target and drive to specified distance using yaw/pitch relative to AprilTag.

### 2. Drive to Field Position
Drive to a 3D field pose. Two implementations:
- **PID-based**: Three PID controllers with stub drivetrain
- **PathPlanner**: On-the-fly command (untested in this package)

### 3. Drive to Target-Relative Position
Drive to position relative to target using `Transform3d` instead of field coordinates.

> [!NOTE]
> Examples target AprilTag #10 (FRC 2025 ReefScape). You'll need to write your own game-specific targeting code.

## Setup

Read the comments in [Robot.java](src/main/java/frc/robot/Robot.java) and each vision system wrapper class for configuration details.

**Required:**
- Your drivetrain implementation
- Odometry
- Gyro (strongly recommended for pose accuracy)

**Defaults:**
- AprilTag #10 (change in code or print tag #10)
- Xbox controller (easily changed to other inputs)

> [!WARNING]
> Tune PID and timeout parameters for your specific robot. Current values are for hand-driven camera testing.

## Hardware Requirements

**Minimum (simulation/testing):**
- DriverStation PC
- USB camera (~$50)

**For competition:**
- Camera with calibration data
- Dedicated coprocessor for vision processing (strongly recommended)

> [!WARNING]
> Running OpenCV directly on the roboRIO (v1 or v2) is discouraged due to substantial CPU and timing tradeoffs. Vision processing should be offloaded to a dedicated coprocessor.

**roboRIO capabilities (if running vision on-RIO):**
- **v1:** Turn-to-angle targeting only; 3D pose consumes nearly all CPU
- **v2:** Can handle 3D pose but still limited for competitive use

## Vision System Options

| System | Cost | Hardware |
|--------|------|----------|
| WPILib ControllerVision | Free | USB camera only |
| PhotonVision | Free | USB camera + coprocessor (recommended) |
| LimelightVision | Purchase required | All-in-one device |

PhotonVision on a coprocessor offers good quality at low cost. LimelightVision is simpler to set up but more expensive.

## Running

Simulation mode works with all three vision systems on roboRIO v1.

> [!NOTE]
> LimelightVision in simulation requires connecting your PC and Limelight to the same network.