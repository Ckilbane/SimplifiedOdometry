# SPOT: Simplified Pinpoint Odometry Tracker

SPOT is [Phil Malone's Simplified Odometry](../readme.md) rebuilt for the **goBILDA Pinpoint Odometry Computer**. It keeps Phil's idea: simple, straight-line moves (`drive`, `strafe`, `turnTo`) that measure what the robot actually did, instead of a full path-following library. The Pinpoint takes the place of the two dead-wheel encoder ports and the Control Hub IMU.

**Download page and setup guide:** https://ckilbane.github.io/SimplifiedOdometry/

SPOT comes in three versions:

| Folder | Robot | Language | Start here if... |
|---|---|---|---|
| [`mecanum/`](mecanum/) | Mecanum + Pinpoint (built on a goBILDA Strafer) | OnBot Java | your team writes Java |
| [`starterbot/`](starterbot/) | goBILDA StarterBot (tank drive) + Pinpoint | OnBot Java | you run a 2-motor tank drive |
| [`blocks/`](blocks/) | Mecanum + Pinpoint | **Blocks** | your team or mentors only use Blocks |

All of it targets **FTC SDK 10.3 or newer**, which has the Pinpoint driver built in. No extra driver files are needed.

## What changed from Phil's original

- **Pinpoint instead of dead wheels + IMU.** `readSensors()` reads position and heading from the Pinpoint, which fuses its two pods with its own IMU at about 1500 Hz.
- **Distances stay robot-relative after a turn.** The Pinpoint reports field position. Phil's moves expect distance along the robot's own axes. SPOT converts the Pinpoint's position using the heading the robot had when each move started, so `drive()` and `strafe()` still measure the right direction after a `turnTo()`.
- **Explicit units.** Every Pinpoint call passes a `DistanceUnit` or `AngleUnit`, as the SDK's built-in driver requires.
- **Standard hardware names** (mecanum and Blocks versions):

  | Device | Config name |
  |---|---|
  | Drive motors | `drive_leftfront_motor`, `drive_rightfront_motor`, `drive_leftback_motor`, `drive_rightback_motor` |
  | Pinpoint (I2C, "goBILDA Pinpoint Odometry Computer") | `odom_pinpoint_sensor` |

  The StarterBot version keeps the StarterBot kit's names (`left_drive`, `right_drive`, `odo`) so it drops onto a kit robot without reconfiguring.

`ProportionalControl` and the motion logic are unchanged from Phil's code.

## mecanum/ (OnBot Java)

| File | What it is |
|---|---|
| `SimplifiedOdometryRobot.java` | The robot class. `drive()`, `strafe()`, `turnTo()`, `resetHeading()`. |
| `MotorDirectionTest.java` | Spins each wheel so you can set motor directions. |
| `PinpointTest.java` | Shows Pinpoint X, Y and heading so you can check pod directions and offsets. |
| `FieldCentricTeleOp.java` | Field-centric driving using the Pinpoint's heading. |
| `FirstAuto.java` | A 24-inch square, the first autonomous to run. |

Upload all five through OnBot Java and click **Build Everything**. Then set the pod offsets, pod type and encoder directions at the top of `SimplifiedOdometryRobot.java`. Run them in order: Motor Direction Test, Pinpoint Test, the TeleOp, then First Auto.

This replaces Phil's root-level `SimplifiedOdometryRobot.java`. Don't put both on the same Control Hub, because they share a class name.

## starterbot/ (OnBot Java)

`StarterBotOdometryRobot.java` is the tank-drive version: `drive()` and `turnTo()`, no strafing. It keeps its own copy of `ProportionalControl` inside the class, so it can sit on the same Control Hub as the mecanum version.

## blocks/ (Blocks)

`SpotBlocks.java` turns SPOT into custom Blocks (myBlocks). A team pastes this one file into OnBot Java once, then works entirely in Blocks under **Java Classes → SpotBlocks**. Every setting is a block: motor directions, pod type and offsets, pod directions, gains and device names. Mentors never have to edit Java.

Open [`SPOT-Blocks-Guide.html`](blocks/SPOT-Blocks-Guide.html) in a browser for the full setup. It covers configuration, install, a Motor Direction Check and a Pinpoint Check built in Blocks, first autonomous programs, tuning and error messages. Fill in the "Your robot" card and the block pictures update with your values.

The blocks stop with a plain-language message on the Driver Station if they're used out of order. Examples are a move block before `init robot`, a setup block after it, a power of 0, or a misspelled pod type.

## Status

- Every file compiles together against stand-ins for the SDK classes. Nothing here has been built against the real SDK.
- The mecanum Java files are the version used for the SCH Strafer drivetrain lab.
- The Blocks version has not run on a robot yet. Reports from the first teams that try it are welcome.

## Credit

Simplified Odometry and `ProportionalControl` are by Phil Malone (Mr. Phil). Watch his four-part video series linked in the [main readme](../readme.md); everything there still applies to SPOT. The Pinpoint conversion and the Blocks version are by Corey Kilbane, Springside Chestnut Hill Academy robotics.
