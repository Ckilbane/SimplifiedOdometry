/* First Auto — Phase 5 of the drivetrain setup guide.
 *
 * Drives a 24-inch square: forward, turn left, repeat four times.
 * Ends where it started, facing the same way.
 * Needs about 3 x 3 field tiles of clear space, starting in the bottom-right one.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// preselectTeleOp loads the TeleOp as soon as this auto ends, ready to continue from here.
@Autonomous(name = "First Auto: Square", group = "Auto", preselectTeleOp = "Field Centric Drive (Pinpoint)")
public class FirstAuto extends LinearOpMode {

    // All the drive code lives in SimplifiedOdometryRobot.
    // "this" hands it our OpMode so it can read the hardware map and telemetry.
    private final SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    @Override
    public void runOpMode() {

        // Which way the robot faces at the start: 0 = away from the drivers,
        // 90 = toward their left, -90 = their right. TeleOp uses this to know which
        // way is "away from the driver". turnTo() still measures from the start facing.
        robot.setStartHeading(0);

        // Connect the hardware and set up the Pinpoint. Robot must be still.
        robot.initialize(true);

        // Show live position while we wait, so you can see the Pinpoint is working.
        while (opModeInInit()) {
            robot.readSensors();
            telemetry.addLine("Ready. Press START to run the square.");
            telemetry.update();
        }

        // Whatever way the robot faces right now is heading 0.
        robot.resetHeading();

        // ------------------------------------------------------------------
        //  The path. Each line finishes before the next one starts.
        //
        //  robot.drive (inches,  maxPower, holdSeconds)   + forward  / - back
        //  robot.strafe(inches,  maxPower, holdSeconds)   + left     / - right
        //  robot.turnTo(degrees, maxPower, holdSeconds)   + left/CCW / - right/CW
        //
        //  turnTo uses ABSOLUTE headings: 90 means "face 90 deg left of where
        //  we started", not "turn 90 more".
        // ------------------------------------------------------------------

        robot.drive(24, 0.6, 0.25);     // side 1
        robot.turnTo(90, 0.45, 0.5);

        robot.drive(24, 0.6, 0.25);     // side 2
        robot.turnTo(180, 0.45, 0.5);

        robot.drive(24, 0.6, 0.25);     // side 3
        robot.turnTo(-90, 0.45, 0.5);   // same as 270

        robot.drive(24, 0.6, 0.25);     // side 4
        robot.turnTo(0, 0.45, 0.5);     // back to the start heading

        telemetry.addLine("Square done.");
        telemetry.update();
        sleep(1000);
    }
}
