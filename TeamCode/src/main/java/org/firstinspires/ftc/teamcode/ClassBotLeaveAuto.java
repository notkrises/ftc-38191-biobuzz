package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * First autonomous for the Class Bot: drive forward for a set time, then stop.
 * In a match this earns LEAVE (3 points): the robot must stop touching the wall.
 *
 * It's "timed": the robot drives at DRIVE_POWER for DRIVE_SECONDS. How far that goes
 * depends on the battery and the floor, so adjust DRIVE_SECONDS after testing.
 *
 * The motors run in encoder mode, so both sides turn at the same speed and the robot
 * drives straight. This needs each drive motor's encoder cable plugged into the hub's
 * encoder port with the same number as its motor port.
 *
 * Configuration names: left_drive, right_drive (same as Class Bot Drive).
 */
@Autonomous(name = "Class Bot Leave", group = "Class Bot", preselectTeleOp = "Class Bot Drive")
public class ClassBotLeaveAuto extends LinearOpMode {

    // How hard and how long to drive. Tune these on the field.
    private static final double DRIVE_POWER = 0.5;
    private static final double DRIVE_SECONDS = 1.0;

    // If a wheel hasn't turned at least this many encoder counts after ENCODER_CHECK_SECONDS,
    // its encoder isn't working, and encoder mode would run that motor at full power. Stop instead.
    private static final double ENCODER_CHECK_SECONDS = 0.3;
    private static final int ENCODER_MIN_COUNTS = 10;

    @Override
    public void runOpMode() {
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Same directions as Class Bot Drive, so positive power means forward.
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // BRAKE makes the robot stop right away instead of coasting past where it should stop.
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset the encoder counts to 0, then switch to encoder mode. In encoder mode,
        // setPower(0.5) means "half speed" instead of "half power": each motor measures its
        // own speed and adjusts, so a side with more friction pushes harder to keep up.
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Ready: will drive forward for %.1f seconds", DRIVE_SECONDS);
        telemetry.update();

        // Nothing moves until the driver presses START.
        waitForStart();

        // Start a stopwatch and turn both motors on.
        ElapsedTime timer = new ElapsedTime();
        leftDrive.setPower(DRIVE_POWER);
        rightDrive.setPower(DRIVE_POWER);

        // Keep driving until the time is up, or until the driver presses STOP.
        String problem = null;
        while (opModeIsActive() && timer.seconds() < DRIVE_SECONDS) {
            int leftCounts = leftDrive.getCurrentPosition();
            int rightCounts = rightDrive.getCurrentPosition();

            // Safety check: by now both wheels should have turned. If one hasn't,
            // its encoder isn't plugged in, so stop before that motor runs away.
            if (timer.seconds() > ENCODER_CHECK_SECONDS) {
                if (Math.abs(leftCounts) < ENCODER_MIN_COUNTS) {
                    problem = "LEFT encoder isn't counting. Check its encoder cable.";
                } else if (Math.abs(rightCounts) < ENCODER_MIN_COUNTS) {
                    problem = "RIGHT encoder isn't counting. Check its encoder cable.";
                }
                if (problem != null) {
                    break;
                }
            }

            telemetry.addData("Driving", "%.1f of %.1f seconds", timer.seconds(), DRIVE_SECONDS);
            telemetry.addData("Encoders", "left %d, right %d", leftCounts, rightCounts);
            telemetry.update();
        }

        // Time's up (or something went wrong): stop both motors.
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        if (problem != null) {
            telemetry.addData("STOPPED", problem);
        } else {
            telemetry.addData("Status", "Done");
        }
        // Final counts: if they're very different, one side still turned more than the other.
        telemetry.addData("Encoders", "left %d, right %d",
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.update();

        // Keep the message on screen until the driver presses STOP.
        while (opModeIsActive()) {
            idle();
        }
    }
}
