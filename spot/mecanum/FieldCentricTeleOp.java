/* Field-Centric TeleOp using the goBILDA Pinpoint Odometry Computer.
 *
 * The Pinpoint reports the robot's heading. We use it to rotate the driver's
 * stick into the robot's frame, so pushing the stick away from you always
 * drives the robot away from you, whichever way the robot is facing.
 *
 * Controls (Gamepad 1):
 *   Left Stick      Translate (field-relative: up = away from the driver)
 *   Right Stick X   Rotate (right = clockwise)
 *   Left Bumper     Hold for slow/precision mode
 *   Y Button        Zero heading. Point the robot away from you first.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

@TeleOp(name = "Field Centric Drive (Pinpoint)", group = "Drive")
public class FieldCentricTeleOp extends LinearOpMode {

    // Speed multiplier while holding the left bumper.
    private static final double SLOW_MODE_SCALAR  = 0.35;

    // Stick values smaller than this count as zero (stops creep from stick drift).
    private static final double JOYSTICK_DEADBAND = 0.05;

    // Mecanum strafing is less efficient than driving forward.
    // Boosting lateral input ~10% makes both feel about the same.
    private static final double STRAFE_CORRECTION = 1.1;

    // Pod offsets in mm, measured from the robot's center of rotation.
    // X offset: how far LEFT of center the X (forward) pod is. Right of center = negative.
    // Y offset: how far FORWARD of center the Y (strafe) pod is. Behind center = negative.
    private static final double POD_OFFSET_X_MM = 0.0;
    private static final double POD_OFFSET_Y_MM = 0.0;

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {

        // --- Drive motors (directions from the Motor Direction Test) ---
        leftFrontDrive  = setupDriveMotor("drive_leftfront_motor",  DcMotor.Direction.REVERSE);
        rightFrontDrive = setupDriveMotor("drive_rightfront_motor", DcMotor.Direction.FORWARD);
        leftBackDrive   = setupDriveMotor("drive_leftback_motor",   DcMotor.Direction.REVERSE);
        rightBackDrive  = setupDriveMotor("drive_rightback_motor",  DcMotor.Direction.FORWARD);

        // --- Pinpoint ---
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odom_pinpoint_sensor");
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

        // --- Bulk caching for faster hub reads ---
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addData("Status",   "Initialized. Press START.");
        telemetry.addData("Pinpoint", odo.getDeviceStatus());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // One fresh read from the Pinpoint per loop.
            odo.update();

            // Heading in radians, counter-clockwise positive.
            double botHeading = odo.getHeading(AngleUnit.RADIANS);

            // --- Heading reset ---
            // Pressing Y makes the robot's current facing the new "away from driver".
            if (gamepad1.y) {
                odo.setPosition(new Pose2D(
                        DistanceUnit.MM,
                        odo.getPosX(DistanceUnit.MM),
                        odo.getPosY(DistanceUnit.MM),
                        AngleUnit.RADIANS, 0
                ));
                botHeading = 0;
            }

            // --- Joystick inputs ---
            // left_stick_y is negative when pushed forward, so negate it.
            double axial   = applyDeadband(-gamepad1.left_stick_y);   // + = away from driver
            double lateral = applyDeadband( gamepad1.left_stick_x);   // + = driver's right
            double yaw     = applyDeadband( gamepad1.right_stick_x);  // + = clockwise

            // --- Slow mode ---
            if (gamepad1.left_bumper) {
                axial   *= SLOW_MODE_SCALAR;
                lateral *= SLOW_MODE_SCALAR;
                yaw     *= SLOW_MODE_SCALAR;
            }

            // --- Field-centric rotation ---
            // Turn the stick direction (field frame) into the robot's own frame.
            // Example: robot turned 90 deg left, driver pushes "away".
            // Away is now on the robot's right, so it should strafe right.
            double cosH = Math.cos(botHeading);
            double sinH = Math.sin(botHeading);

            double rotAxial   = axial * cosH - lateral * sinH;
            double rotLateral = axial * sinH + lateral * cosH;

            rotLateral *= STRAFE_CORRECTION;

            // --- Mecanum mixing (lateral + = right, yaw + = clockwise) ---
            double lF = rotAxial + rotLateral + yaw;
            double rF = rotAxial - rotLateral - yaw;
            double lB = rotAxial - rotLateral + yaw;
            double rB = rotAxial + rotLateral - yaw;

            // Scale all four down together if any is over 1.0.
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

            // --- Telemetry ---
            telemetry.addLine("=== Pinpoint ===");
            telemetry.addData("Heading (deg)",  "%.1f",       Math.toDegrees(botHeading));
            telemetry.addData("Pos X:Y (in)",   "%.2f  %.2f", odo.getPosX(DistanceUnit.INCH), odo.getPosY(DistanceUnit.INCH));
            telemetry.addData("Loop Hz",        "%.0f",       odo.getFrequency());
            telemetry.addData("Status",         odo.getDeviceStatus());
            telemetry.addLine("");
            telemetry.addLine("=== Drive ===");
            telemetry.addData("Powers lF:rF",   "%.2f  %.2f", lF, rF);
            telemetry.addData("Powers lB:rB",   "%.2f  %.2f", lB, rB);
            telemetry.addData("Slow Mode",      gamepad1.left_bumper ? "ON (LB held)" : "off");
            telemetry.addLine("");
            telemetry.addLine("LB = slow mode   Y = zero heading");
            telemetry.update();
        }
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < JOYSTICK_DEADBAND ? 0.0 : value;
    }

    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, deviceName);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }
}
