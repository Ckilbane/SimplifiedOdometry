/* SpotBlocks: SPOT (Simplified Pinpoint Odometry Tracker) custom Blocks (myBlocks)
 * for a mecanum robot with a goBILDA Pinpoint.
 *
 * WHAT THIS FILE IS
 *   Mentors and students never need to edit this file. Paste it into OnBot Java once,
 *   click "Build Everything", and a new set of blocks appears in the Blocks editor under
 *   Java Classes > SpotBlocks. Every setting (motor directions, pod type, pod
 *   offsets, gains) is a block, so all tuning happens in Blocks.
 *
 * HOW A BLOCKS PROGRAM USES IT
 *   Before waitForStart:
 *     1. set motor directions      (from the Motor Direction Check)
 *     2. set odometry pods         (pod type and offsets)
 *     3. set pod directions        (from the Pinpoint Check)
 *     4. init robot                (robot must be sitting still)
 *   After waitForStart:
 *     5. reset heading
 *     6. drive / strafe / turn to  (each one finishes before the next block runs)
 *
 * WHERE IT COMES FROM
 *   The motion code is Phil Malone's SimplifiedOdometryRobot (2023), converted to the
 *   goBILDA Pinpoint. drive() and strafe() measure distance along the robot's own axes,
 *   using the heading the robot had when each move started, so moves stay correct
 *   after a turn.
 *
 * Hardware names (ftcmap standard). Your Control Hub configuration must match these
 * exactly, or use the "set device names" block to match your configuration instead:
 *   drive_leftfront_motor, drive_rightfront_motor,
 *   drive_leftback_motor,  drive_rightback_motor,
 *   odom_pinpoint_sensor   (I2C device type: goBILDA Pinpoint Odometry Computer)
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.List;

public class SpotBlocks extends BlocksOpModeCompanion {

    // ===================================================================================
    //  Block colors (hue numbers). Setup blocks are orange, motion blocks are blue,
    //  tuning is purple, and checking blocks are green.
    // ===================================================================================
    private static final int SETUP_COLOR  = 30;
    private static final int MOVE_COLOR   = 210;
    private static final int TUNE_COLOR   = 280;
    private static final int CHECK_COLOR  = 140;

    // Settings collected by the setup blocks. "init robot" uses them, then puts them
    // back to the defaults, so a value left over from an earlier program never sneaks in.
    private static Settings pending = new Settings();
    private static LinearOpMode pendingOwner = null;

    // The robot built by "init robot", and the program that built it.
    private static Robot robot = null;
    private static LinearOpMode robotOwner = null;

    // ===================================================================================
    //  SETUP BLOCKS (use these before "init robot")
    // ===================================================================================

    @ExportToBlocks(
        heading = "set device names",
        color = SETUP_COLOR,
        comment = "Optional. Only needed if your configuration does not use the standard names " +
                  "(drive_leftfront_motor, drive_rightfront_motor, drive_leftback_motor, " +
                  "drive_rightback_motor, odom_pinpoint_sensor). Must go before init robot.",
        tooltip = "Match the names in your Control Hub configuration",
        parameterLabels = {"Left front motor", "Right front motor", "Left back motor",
                           "Right back motor", "Pinpoint"}
    )
    public static void setDeviceNames(String leftFront, String rightFront,
                                      String leftBack, String rightBack, String pinpoint) {
        Settings s = settings();
        s.leftFrontName  = leftFront.trim();
        s.rightFrontName = rightFront.trim();
        s.leftBackName   = leftBack.trim();
        s.rightBackName  = rightBack.trim();
        s.pinpointName   = pinpoint.trim();
    }

    @ExportToBlocks(
        heading = "set motor directions",
        color = SETUP_COLOR,
        comment = "Use the answers from the Motor Direction Check. true = that motor is reversed. " +
                  "A goBILDA Strafer usually needs both LEFT motors reversed. Put it before init robot.",
        tooltip = "Which drive motors are reversed",
        parameterLabels = {"Left front reversed", "Right front reversed",
                           "Left back reversed", "Right back reversed"}
    )
    public static void setMotorDirections(boolean leftFrontReversed, boolean rightFrontReversed,
                                          boolean leftBackReversed, boolean rightBackReversed) {
        Settings s = settings();
        s.leftFrontReversed  = leftFrontReversed;
        s.rightFrontReversed = rightFrontReversed;
        s.leftBackReversed   = leftBackReversed;
        s.rightBackReversed  = rightBackReversed;
        applyToLiveRobot(s);
    }

    @ExportToBlocks(
        heading = "set odometry pods",
        color = SETUP_COLOR,
        comment = "Pod type is the text 4-bar or swingarm. Offsets are in millimeters from the " +
                  "center of the robot: X offset = how far LEFT of center the forward pod is " +
                  "(right of center is negative). Y offset = how far FORWARD of center the " +
                  "sideways pod is (behind center is negative). Put it before init robot.",
        tooltip = "Pod type and where the pods are mounted",
        parameterLabels = {"Pod type", "X pod offset (mm)", "Y pod offset (mm)"}
    )
    public static void setOdometryPods(String podType, double xOffsetMM, double yOffsetMM) {
        Settings s = settings();
        s.swingarmPods = parsePodType(podType);
        s.xOffsetMM    = xOffsetMM;
        s.yOffsetMM    = yOffsetMM;
        applyToLiveRobot(s);
    }

    @ExportToBlocks(
        heading = "set pod directions",
        color = SETUP_COLOR,
        comment = "Use the answers from the Pinpoint Check. X must go UP when the robot is pushed " +
                  "forward. Y must go UP when the robot is pushed left. Set true for any that " +
                  "went the wrong way. Put it before init robot.",
        tooltip = "Which odometry pods count backwards",
        parameterLabels = {"X pod reversed", "Y pod reversed"}
    )
    public static void setPodDirections(boolean xReversed, boolean yReversed) {
        Settings s = settings();
        s.xPodReversed = xReversed;
        s.yPodReversed = yReversed;
        applyToLiveRobot(s);
    }

    @ExportToBlocks(
        heading = "init robot",
        color = SETUP_COLOR,
        comment = "Connects to the motors and Pinpoint and sends the Pinpoint its settings. " +
                  "Put it after the set blocks and before waitForStart. The robot MUST be " +
                  "sitting still, because the Pinpoint calibrates its gyro here.",
        tooltip = "Set up the drive motors and Pinpoint",
        parameterLabels = {"Show telemetry"}
    )
    public static void initRobot(boolean showTelemetry) {
        robot = new Robot(linearOpMode, settings(), showTelemetry);
        robotOwner = linearOpMode;
        pending = new Settings();
        pendingOwner = null;

        // Give the Pinpoint a moment to finish its gyro calibration.
        linearOpMode.sleep(300);
    }

    // ===================================================================================
    //  MOTION BLOCKS (use these after waitForStart)
    //  Each block finishes its move before the next block runs.
    // ===================================================================================

    @ExportToBlocks(
        heading = "reset heading",
        color = MOVE_COLOR,
        comment = "Makes the way the robot is facing right now count as heading 0. " +
                  "Put it right after waitForStart. Turn to blocks measure from here.",
        tooltip = "Current direction becomes heading 0"
    )
    public static void resetHeading() {
        robot().resetHeading();
    }

    @ExportToBlocks(
        heading = "drive",
        color = MOVE_COLOR,
        comment = "Drive forward (positive inches) or backward (negative inches) while holding " +
                  "the current heading. Power is the top speed, 0 to 1. Hold is how many seconds " +
                  "the robot must stay on target before the block finishes (0.25 is a good start).",
        tooltip = "Drive forward or backward a set distance",
        parameterLabels = {"Inches", "Power", "Hold (sec)"}
    )
    public static void drive(double inches, double power, double holdSeconds) {
        checkPower(power, "drive");
        robot().drive(inches, power, holdSeconds);
    }

    @ExportToBlocks(
        heading = "strafe",
        color = MOVE_COLOR,
        comment = "Slide LEFT (positive inches) or RIGHT (negative inches) while holding the " +
                  "current heading. Power is the top speed, 0 to 1. Hold is how many seconds " +
                  "the robot must stay on target before the block finishes.",
        tooltip = "Slide sideways a set distance",
        parameterLabels = {"Inches", "Power", "Hold (sec)"}
    )
    public static void strafe(double inches, double power, double holdSeconds) {
        checkPower(power, "strafe");
        robot().strafe(inches, power, holdSeconds);
    }

    @ExportToBlocks(
        heading = "turn to",
        color = MOVE_COLOR,
        comment = "Turn in place to face a heading in degrees. 0 = the way the robot faced at " +
                  "reset heading. Positive = LEFT (counter-clockwise), negative = RIGHT. This is " +
                  "where to face, not how far to turn: turn to 90 twice only turns once.",
        tooltip = "Turn to face a heading",
        parameterLabels = {"Heading (deg)", "Power", "Hold (sec)"}
    )
    public static void turnTo(double headingDegrees, double power, double holdSeconds) {
        checkPower(power, "turn to");
        robot().turnTo(headingDegrees, power, holdSeconds);
    }

    @ExportToBlocks(
        heading = "stop robot",
        color = MOVE_COLOR,
        comment = "Sets all four drive motors to 0 power.",
        tooltip = "Stop the drive motors"
    )
    public static void stopRobot() {
        robot().stopRobot();
    }

    @ExportToBlocks(
        heading = "heading",
        color = MOVE_COLOR,
        comment = "The robot's heading in degrees, from the last time the Pinpoint was read. " +
                  "Positive = turned left.",
        tooltip = "Robot heading in degrees"
    )
    public static double heading() {
        return robot().heading;
    }

    // ===================================================================================
    //  TUNING BLOCKS
    // ===================================================================================

    @ExportToBlocks(
        heading = "set gains",
        color = TUNE_COLOR,
        comment = "How hard the robot pushes toward its target. Defaults: drive 0.03, " +
                  "strafe 0.03, turn 0.018. If the robot stops short or creeps at the end, " +
                  "raise the gain a little. If it overshoots or wobbles, lower it. Change one " +
                  "number at a time. Works before or after init robot.",
        tooltip = "Tune how hard each move pushes",
        parameterLabels = {"Drive gain", "Strafe gain", "Turn gain"}
    )
    public static void setGains(double driveGain, double strafeGain, double turnGain) {
        Settings s = settings();
        s.driveGain  = driveGain;
        s.strafeGain = strafeGain;
        s.turnGain   = turnGain;

        if (robotIsReady()) {
            robot.driveController.gain  = driveGain;
            robot.strafeController.gain = strafeGain;
            robot.yawController.gain    = turnGain;
        }
    }

    // ===================================================================================
    //  CHECKING BLOCKS
    // ===================================================================================

    @ExportToBlocks(
        heading = "show Pinpoint on Driver Station",
        color = CHECK_COLOR,
        comment = "Reads the Pinpoint and shows X, Y and heading on the Driver Station. Put it " +
                  "inside a repeat while opModeIsActive loop. Push the robot by hand: X should " +
                  "go up going forward, Y should go up going left, heading should go up turning left.",
        tooltip = "Show Pinpoint position on the Driver Station"
    )
    public static void showPinpoint() {
        robot().showPinpoint();
    }

    @ExportToBlocks(
        heading = "show drive telemetry",
        color = CHECK_COLOR,
        comment = "true = show distance, heading and wheel powers on the Driver Station while " +
                  "the robot moves. Handy while tuning. Works only after init robot.",
        tooltip = "Turn movement telemetry on or off",
        parameterLabels = {"Show"}
    )
    public static void showDriveTelemetry(boolean show) {
        robot().showTelemetry = show;
    }

    // ===================================================================================
    //  Helpers (not blocks)
    // ===================================================================================

    // True when "init robot" has run in this program. The Robot Controller reuses the same
    // program object for every run, so this can also be true for a robot built on an earlier
    // run of the same program. Nothing below may treat it as "init robot ran THIS run".
    private static boolean robotIsReady() {
        return robot != null && robotOwner == linearOpMode;
    }

    private static Robot robot() {
        if (!robotIsReady()) {
            throw new IllegalStateException(
                "Put the \"init robot\" block in your program before any drive, strafe, " +
                "turn or show block.");
        }
        return robot;
    }

    // Settings for the program running now. Leftovers from an earlier program are thrown out.
    private static Settings settings() {
        if (pendingOwner != linearOpMode) {
            pending = new Settings();
            pendingOwner = linearOpMode;
        }
        return pending;
    }

    // A power of 0 would leave the robot waiting forever, so catch it with a clear message.
    private static void checkPower(double power, String blockName) {
        if (power <= 0 || power > 1) {
            throw new IllegalArgumentException(
                "The \"" + blockName + "\" block needs a power between 0 and 1 (it got " + power + "). " +
                "Use negative INCHES or HEADING to go the other way, not negative power.");
        }
    }

    // A set block placed after "init robot" still works: the change goes straight to the
    // robot that init robot built. Before init robot (the normal place) there may be a robot
    // left over from an earlier run, because the Robot Controller reuses the same program
    // object every time a program runs. Updating that old robot does no harm, and init robot
    // replaces it a moment later, so any error here is ignored.
    private static void applyToLiveRobot(Settings s) {
        if (!robotIsReady()) return;
        try {
            robot.applySettings(s);
        } catch (RuntimeException ignored) {
            // The old robot's hardware is gone (for example, the configuration changed).
        }
    }

    private static boolean parsePodType(String podType) {
        String t = podType.toLowerCase().replaceAll("[^a-z0-9]", "");
        if (t.contains("swing")) return true;
        if (t.contains("4bar") || t.contains("fourbar") || t.equals("4")) return false;
        throw new IllegalArgumentException(
            "Pod type must be 4-bar or swingarm. The set odometry pods block got: \"" + podType + "\"");
    }

    // ===================================================================================
    //  Settings: everything the setup blocks can change, with default values.
    //  Defaults are the goBILDA Strafer with 4-bar pods mounted at the center.
    // ===================================================================================
    static class Settings {
        String  leftFrontName  = "drive_leftfront_motor";
        String  rightFrontName = "drive_rightfront_motor";
        String  leftBackName   = "drive_leftback_motor";
        String  rightBackName  = "drive_rightback_motor";
        String  pinpointName   = "odom_pinpoint_sensor";

        boolean leftFrontReversed  = true;
        boolean rightFrontReversed = false;
        boolean leftBackReversed   = true;
        boolean rightBackReversed  = false;

        boolean swingarmPods = false;
        double  xOffsetMM    = 0.0;
        double  yOffsetMM    = 0.0;
        boolean xPodReversed = false;
        boolean yPodReversed = false;

        double  driveGain  = 0.03;
        double  strafeGain = 0.03;
        double  turnGain   = 0.018;
    }

    // ===================================================================================
    //  Robot: Phil Malone's SimplifiedOdometryRobot, Pinpoint version.
    // ===================================================================================
    static class Robot {

        // Motion controller settings.
        //   ACCEL     how fast power may ramp up, per second
        //   TOLERANCE how close counts as "there" (inches or degrees)
        //   DEADBAND  error small enough to stop pushing at all
        //   MAX_AUTO  top power for this axis
        // Gains come from Settings so the "set gains" block can change them.
        private static final double DRIVE_ACCEL     = 2.0;
        private static final double DRIVE_TOLERANCE = 0.5;
        private static final double DRIVE_DEADBAND  = 0.2;
        private static final double DRIVE_MAX_AUTO  = 0.6;

        private static final double STRAFE_ACCEL     = 1.5;
        private static final double STRAFE_TOLERANCE = 0.5;
        private static final double STRAFE_DEADBAND  = 0.2;
        private static final double STRAFE_MAX_AUTO  = 0.6;

        private static final double YAW_ACCEL     = 3.0;
        private static final double YAW_TOLERANCE = 1.0;
        private static final double YAW_DEADBAND  = 0.25;
        private static final double YAW_MAX_AUTO  = 0.6;

        double driveDistance  = 0; // along the robot's forward axis this move (+ = forward), inches
        double strafeDistance = 0; // along the robot's left axis this move (+ = left), inches
        double heading        = 0; // degrees (+ = counter-clockwise)
        boolean showTelemetry;

        final ProportionalControl driveController;
        final ProportionalControl strafeController;
        final ProportionalControl yawController;

        private final DcMotor leftFrontDrive;
        private final DcMotor rightFrontDrive;
        private final DcMotor leftBackDrive;
        private final DcMotor rightBackDrive;
        private final GoBildaPinpointDriver odo;

        private final LinearOpMode myOpMode;
        private final ElapsedTime holdTimer = new ElapsedTime();

        private double turnRate            = 0;
        private double moveStartHeadingRad = 0;  // heading when the current drive/strafe began

        Robot(LinearOpMode opMode, Settings s, boolean showTelemetry) {
            myOpMode = opMode;
            this.showTelemetry = showTelemetry;

            driveController  = new ProportionalControl(s.driveGain,  DRIVE_ACCEL,  DRIVE_MAX_AUTO,  DRIVE_TOLERANCE,  DRIVE_DEADBAND,  false);
            strafeController = new ProportionalControl(s.strafeGain, STRAFE_ACCEL, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);
            yawController    = new ProportionalControl(s.turnGain,   YAW_ACCEL,    YAW_MAX_AUTO,    YAW_TOLERANCE,    YAW_DEADBAND,    true);

            leftFrontDrive  = setupDriveMotor(s.leftFrontName,  s.leftFrontReversed);
            rightFrontDrive = setupDriveMotor(s.rightFrontName, s.rightFrontReversed);
            leftBackDrive   = setupDriveMotor(s.leftBackName,   s.leftBackReversed);
            rightBackDrive  = setupDriveMotor(s.rightBackName,  s.rightBackReversed);

            odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, s.pinpointName);
            odo.setOffsets(s.xOffsetMM, s.yOffsetMM, DistanceUnit.MM);
            odo.setEncoderResolution(s.swingarmPods
                    ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
                    : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

            // X must count up when the robot moves forward, Y when it moves left.
            odo.setEncoderDirections(
                    s.xPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                                   : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    s.yPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                                   : GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            // Zero the position and recalibrate the Pinpoint's IMU.
            // The robot MUST be sitting still for this.
            odo.resetPosAndIMU();
            moveStartHeadingRad = 0;

            List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }

        private DcMotor setupDriveMotor(String deviceName, boolean reversed) {
            DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
            aMotor.setDirection(reversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            aMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return aMotor;
        }

        /**
         * Read the Pinpoint and update driveDistance, strafeDistance and heading.
         * Always returns true so it can sit inside a while() condition.
         */
        boolean readSensors() {
            odo.update();

            // The Pinpoint tracks position on the FIELD. drive() and strafe() need distance
            // along the robot's OWN axes, so turn the field position into "how far forward"
            // and "how far left" relative to the heading the robot had when this move started.
            double fieldX = odo.getPosX(DistanceUnit.INCH);
            double fieldY = odo.getPosY(DistanceUnit.INCH);
            double cosH   = Math.cos(moveStartHeadingRad);
            double sinH   = Math.sin(moveStartHeadingRad);

            driveDistance  =  fieldX * cosH + fieldY * sinH;
            strafeDistance = -fieldX * sinH + fieldY * cosH;

            heading  = odo.getHeading(AngleUnit.DEGREES);
            turnRate = odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

            if (showTelemetry) {
                myOpMode.telemetry.addData("Dist Ax:Lat (in)", "%5.2f %5.2f", driveDistance, strafeDistance);
                myOpMode.telemetry.addData("Head Deg:Rate",    "%5.2f %5.2f", heading, turnRate);
                myOpMode.telemetry.addData("Pinpoint Status",  odo.getDeviceStatus());
            }
            return true;
        }

        void drive(double distanceInches, double power, double holdTime) {
            resetOdometry();

            driveController.reset(distanceInches, power);
            strafeController.reset(0);
            yawController.reset();
            holdTimer.reset();

            while (myOpMode.opModeIsActive() && readSensors()) {
                moveRobot(driveController.getOutput(driveDistance),
                          strafeController.getOutput(strafeDistance),
                          yawController.getOutput(heading));

                if (driveController.inPosition() && yawController.inPosition()) {
                    if (holdTimer.time() > holdTime) {
                        break;
                    }
                } else {
                    holdTimer.reset();
                }
                myOpMode.sleep(10);
            }
            stopRobot();
        }

        void strafe(double distanceInches, double power, double holdTime) {
            resetOdometry();

            driveController.reset(0.0);
            strafeController.reset(distanceInches, power);
            yawController.reset();
            holdTimer.reset();

            while (myOpMode.opModeIsActive() && readSensors()) {
                moveRobot(driveController.getOutput(driveDistance),
                          strafeController.getOutput(strafeDistance),
                          yawController.getOutput(heading));

                if (strafeController.inPosition() && yawController.inPosition()) {
                    if (holdTimer.time() > holdTime) {
                        break;
                    }
                } else {
                    holdTimer.reset();
                }
                myOpMode.sleep(10);
            }
            stopRobot();
        }

        void turnTo(double headingDeg, double power, double holdTime) {
            yawController.reset(headingDeg, power);
            holdTimer.reset();

            while (myOpMode.opModeIsActive() && readSensors()) {
                moveRobot(0, 0, yawController.getOutput(heading));

                if (yawController.inPosition()) {
                    if (holdTimer.time() > holdTime) {
                        break;
                    }
                } else {
                    holdTimer.reset();
                }
                myOpMode.sleep(10);
            }
            stopRobot();
        }

        /**
         * Set the four wheel powers for the requested motion.
         * drive + = forward, strafe + = left, yaw + = counter-clockwise.
         */
        void moveRobot(double drive, double strafe, double yaw) {
            double lF = drive - strafe - yaw;
            double rF = drive + strafe + yaw;
            double lB = drive + strafe - yaw;
            double rB = drive - strafe + yaw;

            double max = Math.max(Math.abs(lF), Math.abs(rF));
            max = Math.max(max, Math.abs(lB));
            max = Math.max(max, Math.abs(rB));

            if (max > 1.0) {
                lF /= max;
                rF /= max;
                lB /= max;
                rB /= max;
            }

            leftFrontDrive.setPower(lF);
            rightFrontDrive.setPower(rF);
            leftBackDrive.setPower(lB);
            rightBackDrive.setPower(rB);

            if (showTelemetry) {
                myOpMode.telemetry.addData("Axes D:S:Y",         "%5.2f %5.2f %5.2f", drive, strafe, yaw);
                myOpMode.telemetry.addData("Wheels lf:rf:lb:rb", "%5.2f %5.2f %5.2f %5.2f", lF, rF, lB, rB);
                myOpMode.telemetry.update();
            }
        }

        void stopRobot() {
            moveRobot(0, 0, 0);
        }

        /**
         * Zero the X/Y position for a new move and remember which way the robot faces now.
         * Uses setPosition() instead of resetPosAndIMU(), so it is safe while moving.
         */
        void resetOdometry() {
            odo.update();
            moveStartHeadingRad = odo.getHeading(AngleUnit.RADIANS);
            odo.setPosition(new Pose2D(
                    DistanceUnit.MM, 0, 0,
                    AngleUnit.RADIANS, moveStartHeadingRad
            ));
            driveDistance  = 0.0;
            strafeDistance = 0.0;
            driveController.reset(0);
            strafeController.reset(0);
        }

        /** Make the robot's current facing heading 0. */
        void resetHeading() {
            odo.update();
            odo.setPosition(new Pose2D(
                    DistanceUnit.MM, odo.getPosX(DistanceUnit.MM), odo.getPosY(DistanceUnit.MM),
                    AngleUnit.RADIANS, 0
            ));
            moveStartHeadingRad = 0;
            yawController.reset(0);
            heading = 0;
        }

        /** Send changed directions and pod settings to hardware that is already set up. */
        void applySettings(Settings s) {
            leftFrontDrive.setDirection(s.leftFrontReversed  ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(s.rightFrontReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(s.leftBackReversed   ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(s.rightBackReversed  ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

            odo.setOffsets(s.xOffsetMM, s.yOffsetMM, DistanceUnit.MM);
            odo.setEncoderResolution(s.swingarmPods
                    ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
                    : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(
                    s.xPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                                   : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    s.yPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                                   : GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
        }

        /** Show the Pinpoint's raw field position. Used by the Pinpoint Check program. */
        void showPinpoint() {
            odo.update();
            heading = odo.getHeading(AngleUnit.DEGREES);
            myOpMode.telemetry.addData("X (in)  forward = up", "%6.2f", odo.getPosX(DistanceUnit.INCH));
            myOpMode.telemetry.addData("Y (in)  left = up",    "%6.2f", odo.getPosY(DistanceUnit.INCH));
            myOpMode.telemetry.addData("Heading (deg)  left = up", "%6.1f", heading);
            myOpMode.telemetry.addData("Pinpoint status", odo.getDeviceStatus());
            myOpMode.telemetry.update();
        }
    }

    // ===================================================================================
    //  ProportionalControl: output power is proportional to how far the axis is from its
    //  setpoint, with a ramp-up limit and a top-power limit. Unchanged from the original.
    // ===================================================================================
    static class ProportionalControl {
        double  lastOutput;
        double  gain;
        double  accelLimit;
        double  defaultOutputLimit;
        double  liveOutputLimit;
        double  setPoint;
        double  tolerance;
        double  deadband;
        boolean circular;
        boolean inPosition;
        ElapsedTime cycleTime = new ElapsedTime();

        ProportionalControl(double gain, double accelLimit, double outputLimit,
                            double tolerance, double deadband, boolean circular) {
            this.gain = gain;
            this.accelLimit = accelLimit;
            this.defaultOutputLimit = outputLimit;
            this.liveOutputLimit = outputLimit;
            this.tolerance = tolerance;
            this.deadband = deadband;
            this.circular = circular;
            reset(0.0);
        }

        double getOutput(double input) {
            double error = setPoint - input;
            double dV = cycleTime.seconds() * accelLimit;
            double output;

            if (circular) {
                while (error > 180)   error -= 360;
                while (error <= -180) error += 360;
            }

            inPosition = (Math.abs(error) < tolerance);

            if (Math.abs(error) <= deadband) {
                output = 0;
            } else {
                output = (error * gain);
                output = Range.clip(output, -liveOutputLimit, liveOutputLimit);

                if ((output - lastOutput) > dV) {
                    output = lastOutput + dV;
                } else if ((output - lastOutput) < -dV) {
                    output = lastOutput - dV;
                }
            }

            lastOutput = output;
            cycleTime.reset();
            return output;
        }

        boolean inPosition() { return inPosition; }

        void reset(double setPoint, double powerLimit) {
            liveOutputLimit = Math.abs(powerLimit);
            this.setPoint = setPoint;
            reset();
        }

        void reset(double setPoint) {
            liveOutputLimit = defaultOutputLimit;
            this.setPoint = setPoint;
            reset();
        }

        void reset() {
            cycleTime.reset();
            inPosition = false;
            lastOutput = 0.0;
        }
    }
}
