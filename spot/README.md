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
- **Distances stay robot-relative after a turn.** The Pinpoint reports field position. Phil's moves expect distance along the robot's own axes. Each move records where the robot started and which way it faced, then converts the Pinpoint's movement since then into "how far forward" and "how far left". `drive()` and `strafe()` still measure the right direction after a `turnTo()`.
- **The Pinpoint keeps the field position.** Moves never zero it, so SPOT always knows where the robot is (`getX()`, `getY()`, or the *robot X* / *robot Y* blocks), and can hand that position to TeleOp. See [From auto to TeleOp](#from-auto-to-teleop).
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

The blocks stop with a plain-language message on the Driver Station if they're used wrong, such as a move block before `init robot`, a power of 0, or a misspelled pod type. They also stop the robot if a move goes wrong. If the robot moves away from its target, pushes with no measured progress, or turns off course while driving straight, SpotBlocks stops it and says which pod or motor to check.

## From auto to TeleOp

SPOT saves the robot's position every time it reads the Pinpoint. A TeleOp started within 2 minutes picks up where the autonomous left off. Field centric driving then knows which way is away from the drivers, even if auto ended with the robot turned sideways. Saving on every read means the handoff works even when the 30-second timer stops a move partway through. The saved position lives in memory, so it's lost if the Robot Controller app restarts. In that case TeleOp starts fresh and says so on the Driver Station.

**Start heading.** If the robot doesn't start auto facing away from the drivers, tell SPOT which way it does face, in degrees: 0 = away from the drivers, 90 = toward their left, -90 = their right, 180 = facing them. `turnTo()` doesn't change: `turnTo(0, ...)` still means the way the robot started.

| | Blocks | Mecanum Java | StarterBot Java |
|---|---|---|---|
| Set the start heading (auto only) | *set start heading*, before *init robot* | `robot.setStartHeading(deg)` before `initialize()` | `robot.setStartHeading(deg)` before `initialize()` |
| Continue in TeleOp | *init robot (continue from auto)* | `FieldCentricTeleOp` does it on its own. For your own TeleOp: `robot.initialize(showTelemetry, true)` | `robot.initialize(showTelemetry, true)` |
| Drive field centric | *field centric drive* (plug the sticks straight in) | `FieldCentricTeleOp` | Not needed for tank drive |
| Read the position | *robot X*, *robot Y*, *heading* | `getX()`, `getY()`, `getFieldHeading()` | `getX()`, `getY()`, `getFieldHeading()` |
| Re-aim mid-match | *reset heading* on a button | Y button in `FieldCentricTeleOp`, or `resetHeading()` | `resetHeading()` |

Positions are in the drivers' frame: X = inches away from the drivers, Y = inches toward their left, heading 0 = facing away.

Set each autonomous to preselect its TeleOp, so the Driver Station loads it as soon as auto ends. In Java, `FirstAuto` does it with `preselectTeleOp` in its `@Autonomous` line. In Blocks, it's a dropdown in the autonomous's program editor. The drivers still press INIT and Start.

Each version keeps its own saved position. An autonomous using `SimplifiedOdometryRobot` hands off to a Java TeleOp, and a SpotBlocks autonomous hands off to a SpotBlocks TeleOp.

## Troubleshooting

Robot running away, spinning or stopping with an error? See [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) for the Java versions, or the Troubleshooting section of the Blocks guide.

## Status

- Every file compiles together against stand-ins for the SDK classes. Nothing here has been built against the real SDK.
- The mecanum Java files are the version used for the SCH Strafer drivetrain lab.
- The auto-to-TeleOp handoff has been tested in the simulator for all of these: an auto cut off mid-move, start headings of 0 and 90, a TeleOp restarted mid-match, a TeleOp that copied the auto's start heading, a stale save, and no save at all. That covers the Blocks version and the Java mecanum version.
- The Blocks version is in testing with its first teams. Its safety stops have been tested in a simulator against reversed, swapped and unplugged pods, reversed and swapped motors, and a wall. They haven't been tested on a robot yet.

## Credit

Simplified Odometry and `ProportionalControl` are by Phil Malone (Mr. Phil). Watch his four-part video series linked in the [main readme](../readme.md); everything there still applies to SPOT. The Pinpoint conversion and the Blocks version are by Corey Kilbane, Springside Chestnut Hill Academy robotics.
