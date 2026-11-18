/* Motor Direction Test — Phase 2 of the drivetrain setup guide.
 *
 * Prop the robot up so the wheels are off the ground, then hold one button
 * to spin one wheel slowly:
 *      X = left front     Y = right front
 *      A = left back      B = right back
 *      D-pad up = all four together
 *
 * Every wheel should roll FORWARD (top of the wheel moving toward the front
 * of the robot), and its encoder number should count UP.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Direction Test", group = "Setup")
public class MotorDirectionTest extends LinearOpMode {

    // Low power so you can watch each wheel without the robot running away.
    private static final double TEST_POWER = 0.3;

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    @Override
    public void runOpMode() {

        // !!!  These four directions are what you are testing.
        //      Flip any wheel that rolls backward, build again, and re-test.
        leftFrontDrive  = setupMotor("drive_leftfront_motor",  DcMotor.Direction.REVERSE);
        rightFrontDrive = setupMotor("drive_rightfront_motor", DcMotor.Direction.FORWARD);
        leftBackDrive   = setupMotor("drive_leftback_motor",   DcMotor.Direction.REVERSE);
        rightBackDrive  = setupMotor("drive_rightback_motor",  DcMotor.Direction.FORWARD);

        telemetry.addLine("Wheels off the ground? Press START.");
        telemetry.addLine("Hold X, Y, A or B to spin one wheel.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean all = gamepad1.dpad_up;

            leftFrontDrive.setPower (gamepad1.x || all ? TEST_POWER : 0.0);
            rightFrontDrive.setPower(gamepad1.y || all ? TEST_POWER : 0.0);
            leftBackDrive.setPower  (gamepad1.a || all ? TEST_POWER : 0.0);
            rightBackDrive.setPower (gamepad1.b || all ? TEST_POWER : 0.0);

            telemetry.addLine("Button  Wheel         Encoder");
            telemetry.addData("  X", "left front    %d", leftFrontDrive.getCurrentPosition());
            telemetry.addData("  Y", "right front   %d", rightFrontDrive.getCurrentPosition());
            telemetry.addData("  A", "left back     %d", leftBackDrive.getCurrentPosition());
            telemetry.addData("  B", "right back    %d", rightBackDrive.getCurrentPosition());
            telemetry.addLine("");
            telemetry.addLine("Wheel rolls forward + encoder counts up = correct.");
            telemetry.update();
        }
    }

    private DcMotor setupMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, deviceName);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }
}
