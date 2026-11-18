/* Pinpoint Test — Phase 3 of the drivetrain setup guide.
 *
 * Shows what the Pinpoint thinks the robot is doing, live.
 *   Push the robot by hand, or drive it slowly:
 *     Left stick  = drive and strafe (robot-centric, 40% power)
 *     Right stick = spin
 *     A button    = zero position and heading (robot must be still)
 *
 * Checks:  forward  -> X counts up
 *          left     -> Y counts up
 *          turn CCW -> heading counts up
 *          spin in place -> X and Y stay near 0
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Pinpoint Test", group = "Setup")
public class PinpointTest extends LinearOpMode {

    private static final double DRIVE_SCALE = 0.4;

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

        leftFrontDrive  = setupMotor("drive_leftfront_motor",  DcMotor.Direction.REVERSE);
        rightFrontDrive = setupMotor("drive_rightfront_motor", DcMotor.Direction.FORWARD);
        leftBackDrive   = setupMotor("drive_leftback_motor",   DcMotor.Direction.REVERSE);
        rightBackDrive  = setupMotor("drive_rightback_motor",  DcMotor.Direction.FORWARD);

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

        telemetry.addLine("Pinpoint ready. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            odo.update();

            if (gamepad1.a) {
                odo.resetPosAndIMU();
            }

            // Robot-centric drive, just for testing.
            double axial   = -gamepad1.left_stick_y  * DRIVE_SCALE;  // + = forward
            double lateral =  gamepad1.left_stick_x  * DRIVE_SCALE;  // + = right
            double yaw     =  gamepad1.right_stick_x * DRIVE_SCALE;  // + = clockwise

            leftFrontDrive.setPower (axial + lateral + yaw);
            rightFrontDrive.setPower(axial - lateral - yaw);
            leftBackDrive.setPower  (axial - lateral + yaw);
            rightBackDrive.setPower (axial + lateral - yaw);

            telemetry.addData("X  (in, + = forward)", "%7.2f", odo.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y  (in, + = left)",    "%7.2f", odo.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg, + = CCW)", "%7.1f", odo.getHeading(AngleUnit.DEGREES));
            telemetry.addLine("");
            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("Pinpoint Hz", "%.0f", odo.getFrequency());
            telemetry.addData("Raw encoders X:Y", "%d  %d", odo.getEncoderX(), odo.getEncoderY());
            telemetry.addLine("");
            telemetry.addLine("A = zero (hold robot still)");
            telemetry.update();
        }
    }

    private DcMotor setupMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, deviceName);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }
}
