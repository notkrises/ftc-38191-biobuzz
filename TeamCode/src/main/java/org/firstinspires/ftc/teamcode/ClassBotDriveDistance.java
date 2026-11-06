package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * TEST AUTONOMOUS: drive forward an exact distance using the encoders, then stop.
 * Measure how far it actually went to check COUNTS_PER_INCH.
 *
 * During INIT, D-pad left/right picks the distance: 12, 24, or 48 inches (used for the
 * "Encoder driving accuracy at different distances" lab). The battery voltage is shown too.
 *
 * Driving by distance instead of time means the robot goes the same distance whether the
 * battery is full or low. Once this is accurate, the same method goes into the StarterBot's
 * autonomous programs.
 *
 * Configuration names: left_drive, right_drive (same as Class Bot Drive).
 */
@Autonomous(name = "Class Bot Drive Distance", group = "Class Bot", preselectTeleOp = "Class Bot Drive")
public class ClassBotDriveDistance extends LinearOpMode {

    // Encoder counts per inch of travel, for the Class Bot. REV's spec sheet suggests 47.5;
    // on 10/29 we tried 60 and the robot went 18.5 in instead of 24, so 60 x 24 / 18.5 = 77.8.
    // 77.8 went 0.5-1 in over every time (about 24.75 in), so 77.8 x 24 / 24.75 = 75.4.
    // To fine-tune: run this, measure how far it really went, then
    //   new value = (current value) x 24 / (inches it actually went)
    // Too short = raise it. Too far = lower it.
    private static final double COUNTS_PER_INCH = 75.4;

    // The distances the D-pad can pick during INIT. Starts on 24 in.
    private static final double[] DISTANCES_INCHES = {12.0, 24.0, 48.0};
    private static final int START_CHOICE = 1;
    private static final double DRIVE_POWER = 0.4;
    // If it hasn't arrived after this long, something's wrong (like an unplugged encoder): stop.
    // Long enough for the 48 in drive.
    private static final double TIMEOUT_SECONDS = 8.0;

    @Override
    public void runOpMode() {
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // During INIT: pick the distance with the D-pad, and show the battery voltage so it can be
        // recorded for the lab. The "WasDown" flags make one press move one step.
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        int choice = START_CHOICE;
        boolean leftWasDown = false;
        boolean rightWasDown = false;
        while (opModeInInit()) {
            if (gamepad1.dpad_right && !rightWasDown) {
                choice = Math.min(choice + 1, DISTANCES_INCHES.length - 1);
            }
            if (gamepad1.dpad_left && !leftWasDown) {
                choice = Math.max(choice - 1, 0);
            }
            rightWasDown = gamepad1.dpad_right;
            leftWasDown = gamepad1.dpad_left;

            telemetry.addData("Distance", "%.0f in  (D-pad left/right: 12, 24, 48)", DISTANCES_INCHES[choice]);
            telemetry.addData("Battery", "%.2f V  <- record this", battery.getVoltage());
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }

        driveInches(leftDrive, rightDrive, DISTANCES_INCHES[choice]);

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
