package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Autonomous for the main robot (goBILDA StarterBot): drive forward for a set time, then stop.
 * In a match this earns LEAVE (3 points): the robot must stop touching the wall.
 *
 * It's "timed": the robot drives at DRIVE_POWER for DRIVE_SECONDS. How far that goes
 * depends on the battery and the floor, so adjust DRIVE_SECONDS after testing.
 * It uses plain power (not encoder mode) so it works no matter how the encoders are wired.
 *
 * Set the robot down with its FRONT facing away from the wall.
 * Configuration names: left_drive, right_drive (same as StarterBot Drive).
 */
@Autonomous(name = "StarterBot Leave", group = "StarterBot", preselectTeleOp = "StarterBot Drive")
public class StarterBotLeaveAuto extends LinearOpMode {

    // How hard and how long to drive. Tune these on the field.
    private static final double DRIVE_POWER = 0.5;
    private static final double DRIVE_SECONDS = 1.0;

    @Override
    public void runOpMode() {
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Same directions as StarterBot Drive (the StarterBot's gears flip them compared
        // with the Class Bot), so positive power means forward.
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Plain power mode, set on purpose: the motor mode stays on the hub between programs.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // BRAKE makes the robot stop right away instead of coasting past where it should stop.
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready: will drive forward for %.1f seconds", DRIVE_SECONDS);
        telemetry.update();

        // Nothing moves until the driver presses START.
        waitForStart();

        // Start a stopwatch and turn both motors on.
        ElapsedTime timer = new ElapsedTime();
        leftDrive.setPower(DRIVE_POWER);
        rightDrive.setPower(DRIVE_POWER);

        // Keep driving until the time is up, or until the driver presses STOP.
        while (opModeIsActive() && timer.seconds() < DRIVE_SECONDS) {
            telemetry.addData("Driving", "%.1f of %.1f seconds", timer.seconds(), DRIVE_SECONDS);
            telemetry.update();
        }

        // Time's up: stop both motors.
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        telemetry.addData("Status", "Done");
        telemetry.update();
    }
}
