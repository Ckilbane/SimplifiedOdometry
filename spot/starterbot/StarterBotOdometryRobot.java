/* StarterBot version of SimplifiedOdometryRobot (Phil Malone, 2023),
 * modified for the goBILDA StarterBot with a goBILDA Pinpoint Odometry Computer.
 *
 * The StarterBot is tank (skid-steer) drive: two motors, no strafing.
 * So this class has drive() and turnTo(), but no strafe().
 * The Pinpoint gives us distance and heading, so drive() holds a straight
 * heading the whole way, and turnTo() uses the Pinpoint's IMU instead of
 * counting wheel ticks.
 *
 * Plans made in the BioBuzz Path Planner with "StarterBot + Pinpoint"
 * selected call this class. Save it as StarterBotOdometryRobot.java in OnBot Java.
 *
 * Hardware names (Driver Station config):
 *   left_drive, right_drive   goBILDA 5203 312 RPM motors (same as the StarterBot kit)
 *   odo                       goBILDA Pinpoint Odometry Computer on Control Hub I2C bus 0
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.List;

public class StarterBotOdometryRobot {

    // ===== Robot constants (Copy robot constants in the planner's Setup tab gives this block) =====

    // Pod offsets in mm, measured from the robot's center of rotation.
    // X offset: how far LEFT of center the X (forward) pod is. Right of center = negative.
    // Y offset: how far FORWARD of center the Y (strafe) pod is. Behind center = negative.
    private static final double POD_OFFSET_X_MM = 0.0;
    private static final double POD_OFFSET_Y_MM = 0.0;

    // Motion controller settings.
    //   GAIN      how hard to push per inch (or degree) of error
    //   ACCEL     how fast power may ramp up, per second
    //   TOLERANCE how close counts as "there" (inches or degrees)
    //   DEADBAND  error small enough to stop pushing at all
    //   MAX_AUTO  top power for this axis in autonomous
    private static final double DRIVE_GAIN          = 0.03;
    private static final double DRIVE_ACCEL         = 2.0;
    private static final double DRIVE_TOLERANCE     = 0.5;
    private static final double DRIVE_DEADBAND      = 0.2;
    private static final double DRIVE_MAX_AUTO      = 0.6;

    private static final double YAW_GAIN            = 0.018;
    private static final double YAW_ACCEL           = 3.0;
    private static final double YAW_TOLERANCE       = 1.0;
    private static final double YAW_DEADBAND        = 0.25;
    private static final double YAW_MAX_AUTO        = 0.6;

    // Public members
    public double driveDistance = 0; // distance along the robot's forward axis this move (+ = forward), inches
    public double sideSlip      = 0; // sideways distance this move (+ = left), inches. Tank drive can't fix it; it's shown so you can see it.
    public double heading       = 0; // robot heading, degrees (+ = counter-clockwise)

    public ProportionalControl driveController = new ProportionalControl(DRIVE_GAIN, DRIVE_ACCEL, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    public ProportionalControl yawController   = new ProportionalControl(YAW_GAIN,   YAW_ACCEL,   YAW_MAX_AUTO,   YAW_TOLERANCE,   YAW_DEADBAND,   true);

    // Hardware
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    private GoBildaPinpointDriver odo;

    private LinearOpMode myOpMode;
    private ElapsedTime holdTimer = new ElapsedTime();

    private double turnRate            = 0;
    private double moveStartHeadingRad = 0;  // heading when the current drive began
    private boolean showTelemetry      = false;

    public StarterBotOdometryRobot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Connect to the hardware and send the Pinpoint its settings.
     * The robot must be sitting still when this runs.
     * @param showTelemetry  true to show sensor and drive values on the Driver Station.
     */
    public void initialize(boolean showTelemetry) {

        // Same directions as the goBILDA StarterBot code. Check them with the
        // Motor Direction Test: both wheels must roll forward on positive power.
        leftDrive  = setupDriveMotor("left_drive",  DcMotor.Direction.FORWARD);
        rightDrive = setupDriveMotor("right_drive", DcMotor.Direction.REVERSE);

        // Pinpoint is plugged into I2C bus 0, configured as "goBILDA Pinpoint Odometry Computer" named "odo".
        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(POD_OFFSET_X_MM, POD_OFFSET_Y_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // X must count up when the robot moves forward, Y when it moves left.
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Zero the position and recalibrate the Pinpoint's IMU.
        // The robot MUST be sitting still for this (about a quarter second).
        odo.resetPosAndIMU();
        moveStartHeadingRad = 0;

        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.showTelemetry = showTelemetry;
    }

    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return aMotor;
    }

    /**
     * Read the Pinpoint and update driveDistance, sideSlip and heading.
     * Always returns true so it can sit inside a while() condition.
     */
    public boolean readSensors() {
        odo.update();

        // The Pinpoint tracks position on the FIELD. drive() needs distance along the
        // robot's OWN forward axis, so we turn the field position into "how far forward"
        // relative to the heading the robot had when this move started.
        double fieldX = odo.getPosX(DistanceUnit.INCH);
        double fieldY = odo.getPosY(DistanceUnit.INCH);
        double cosH   = Math.cos(moveStartHeadingRad);
        double sinH   = Math.sin(moveStartHeadingRad);

        driveDistance =  fieldX * cosH + fieldY * sinH;
        sideSlip      = -fieldX * sinH + fieldY * cosH;

        heading  = odo.getHeading(AngleUnit.DEGREES);
        turnRate = odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        if (showTelemetry) {
            myOpMode.telemetry.addData("Dist Fwd:Slip (in)", "%5.2f %5.2f", driveDistance, sideSlip);
            myOpMode.telemetry.addData("Head Deg:Rate",      "%5.2f %5.2f", heading, turnRate);
            myOpMode.telemetry.addData("Pinpoint Status",    odo.getDeviceStatus());
        }
        return true;
    }

    //  ########################  Mid level control functions.  ##############################

    /**
     * Drive forward or backward while holding heading.
     * @param distanceInches  + = forward, - = backward.
     * @param power           Top power, always positive.
     * @param holdTime        Seconds to hold the final position. 0 = no hold.
     */
    public void drive(double distanceInches, double power, double holdTime) {
        resetOdometry();

        driveController.reset(distanceInches, power);
        yawController.reset();
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()) {
            moveRobot(driveController.getOutput(driveDistance), yawController.getOutput(heading));

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

    /**
     * Tank drive can't strafe. This is here so a plan with a strafe step still
     * compiles; it does nothing except say so on the Driver Station.
     */
    public void strafe(double distanceInches, double power, double holdTime) {
        myOpMode.telemetry.addLine("strafe() skipped: the StarterBot can't strafe.");
        myOpMode.telemetry.update();
    }

    /**
     * Turn in place to an absolute heading.
     * @param headingDeg  0 = the way the robot faced at START. + = left (CCW), - = right (CW).
     * @param power       Top power, always positive.
     * @param holdTime    Seconds to hold the final heading. 0 = no hold.
     */
    public void turnTo(double headingDeg, double power, double holdTime) {
        yawController.reset(headingDeg, power);
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()) {
            moveRobot(0, yawController.getOutput(heading));

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

    //  ########################  Low level control functions.  ###############################

    /**
     * Set the two wheel powers for the requested motion.
     * @param drive  + = forward
     * @param yaw    + = counter-clockwise (the right wheel speeds up, the left slows down)
     */
    public void moveRobot(double drive, double yaw) {
        double left  = drive - yaw;
        double right = drive + yaw;

        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left  /= max;
            right /= max;
        }

        leftDrive.setPower(left);
        rightDrive.setPower(right);

        if (showTelemetry) {
            myOpMode.telemetry.addData("Axes D:Y",     "%5.2f %5.2f", drive, yaw);
            myOpMode.telemetry.addData("Wheels L:R",   "%5.2f %5.2f", left, right);
            myOpMode.telemetry.update();
        }
    }

    public void stopRobot() {
        moveRobot(0, 0);
    }

    /**
     * Zero the X/Y position for a new move and remember which way the robot faces now.
     * Uses setPosition() instead of resetPosAndIMU(), so it is safe while moving.
     */
    public void resetOdometry() {
        odo.update();
        moveStartHeadingRad = odo.getHeading(AngleUnit.RADIANS);
        odo.setPosition(new Pose2D(
                DistanceUnit.MM, 0, 0,
                AngleUnit.RADIANS, moveStartHeadingRad
        ));
        driveDistance = 0.0;
        sideSlip      = 0.0;
        driveController.reset(0);
    }

    /**
     * Make the robot's current facing heading 0. Call this right after START.
     */
    public void resetHeading() {
        odo.update();
        odo.setPosition(new Pose2D(
                DistanceUnit.MM, odo.getPosX(DistanceUnit.MM), odo.getPosY(DistanceUnit.MM),
                AngleUnit.RADIANS, 0
        ));
        moveStartHeadingRad = 0;
        yawController.reset(0);
        heading = 0;
    }

    public double getHeading()  { return heading; }
    public double getTurnRate() { return turnRate; }

    public void showTelemetry(boolean show) {
        showTelemetry = show;
    }

    //****************************************************************************************************

    /***
     * A proportional controller: output power is proportional to how far the axis
     * is from its setpoint, with a ramp-up limit and a top-power limit.
     * Same as the one in SimplifiedOdometryRobot. It lives inside this class so the
     * two files can sit in the same OnBot Java project without a name clash.
     */
    public static class ProportionalControl {
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

        public ProportionalControl(double gain, double accelLimit, double outputLimit, double tolerance, double deadband, boolean circular) {
            this.gain = gain;
            this.accelLimit = accelLimit;
            this.defaultOutputLimit = outputLimit;
            this.liveOutputLimit = outputLimit;
            this.tolerance = tolerance;
            this.deadband = deadband;
            this.circular = circular;
            reset(0.0);
        }

        public double getOutput(double input) {
            double error = setPoint - input;
            double dV = cycleTime.seconds() * accelLimit;
            double output;

            if (circular) {
                while (error > 180)  error -= 360;
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

        public boolean inPosition()  { return inPosition; }
        public double  getSetpoint() { return setPoint; }

        public void reset(double setPoint, double powerLimit) {
            liveOutputLimit = Math.abs(powerLimit);
            this.setPoint = setPoint;
            reset();
        }

        public void reset(double setPoint) {
            liveOutputLimit = defaultOutputLimit;
            this.setPoint = setPoint;
            reset();
        }

        public void reset() {
            cycleTime.reset();
            inPosition = false;
            lastOutput = 0.0;
        }
    }
}
