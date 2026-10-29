package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * TEST AUTONOMOUS: drive forward an exact distance using the encoders, then stop.
 * Measure how far it actually went to check COUNTS_PER_INCH.
 *
 * Driving by distance instead of time means the robot goes the same distance whether the
 * battery is full or low. Once this is accurate, the same method goes into the StarterBot's
 * autonomous programs.
 *
 * Configuration names: left_drive, right_drive (same as Class Bot Drive).
 */
@Autonomous(name = "Class Bot Drive 24 in", group = "Class Bot", preselectTeleOp = "Class Bot Drive")
public class ClassBotDriveDistance extends LinearOpMode {

    // Encoder counts per inch of travel, for the Class Bot. REV's spec sheet suggests 47.5;
    // on 10/29 we tried 60 and the robot went 18.5 in instead of 24, so 60 x 24 / 18.5 = 77.8.
    // 77.8 went 0.5-1 in over every time (about 24.75 in), so 77.8 x 24 / 24.75 = 75.4.
    // To fine-tune: run this, measure how far it really went, then
    //   new value = (current value) x 24 / (inches it actually went)
    // Too short = raise it. Too far = lower it.
    private static final double COUNTS_PER_INCH = 75.4;

    private static final double DISTANCE_INCHES = 24.0;
    private static final double DRIVE_POWER = 0.4;
    // If it hasn't arrived after this long, something's wrong (like an unplugged encoder): stop.
    private static final double TIMEOUT_SECONDS = 5.0;

    @Override
    public void runOpMode() {
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Ready", "will drive %.0f inches", DISTANCE_INCHES);
        telemetry.update();
        waitForStart();

        driveInches(leftDrive, rightDrive, DISTANCE_INCHES);

        // Put the motors back in normal mode. A motor's mode stays on the robot after a program
        // ends, and Class Bot Drive can't be driven while the motors are in RUN_TO_POSITION.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            idle();
        }
    }

    /**
     * Drive straight a set number of inches (negative = backward).
     * RUN_TO_POSITION: each motor drives itself to a target encoder count and stops there.
     */
    private void driveInches(DcMotor leftDrive, DcMotor rightDrive, double inches) {
        int targetCounts = (int) Math.round(inches * COUNTS_PER_INCH);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setTargetPosition(targetCounts);
        rightDrive.setTargetPosition(targetCounts);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime timer = new ElapsedTime();
        leftDrive.setPower(DRIVE_POWER);
        rightDrive.setPower(DRIVE_POWER);

        // isBusy() is true while a motor is still on its way to the target.
        while (opModeIsActive() && timer.seconds() < TIMEOUT_SECONDS
                && (leftDrive.isBusy() || rightDrive.isBusy())) {
            telemetry.addData("Target", "%d counts (%.0f in)", targetCounts, inches);
            telemetry.addData("Left", leftDrive.getCurrentPosition());
            telemetry.addData("Right", rightDrive.getCurrentPosition());
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        boolean timedOut = timer.seconds() >= TIMEOUT_SECONDS;
        telemetry.addData("Status", timedOut
                ? "STOPPED: didn't arrive in time. Check both encoder cables."
                : "Arrived. Now measure how far it actually went.");
        telemetry.addData("Final counts", "left %d, right %d (target %d)",
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition(), targetCounts);
        telemetry.update();
    }
}
