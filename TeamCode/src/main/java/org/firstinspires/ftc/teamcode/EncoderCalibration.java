package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * TEST PROGRAM: measures how many encoder counts the drive wheels make per inch.
 * Works on any robot whose drive motors are named left_drive and right_drive.
 *
 * How to use:
 *   1. Lay a tape measure on the floor next to the robot.
 *   2. Press START (the motors are off; the robot can be pushed by hand).
 *   3. Press Y to set the counts to zero.
 *   4. PUSH the robot straight forward exactly 24 inches along the tape.
 *   5. Read "Counts per inch" on the screen. That number goes into the drive-by-distance code.
 * Do it 2 or 3 times and use the average.
 */
@TeleOp(name = "Encoder Calibration", group = "Test")
public class EncoderCalibration extends LinearOpMode {

    private static final double PUSH_DISTANCE_INCHES = 24.0;

    @Override
    public void runOpMode() {
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // FLOAT lets the wheels roll freely, so the robot is easy to push by hand.
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        resetCounts(leftDrive, rightDrive);

        telemetry.addData("Ready", "Press START, then Y to zero, then push the robot 24 inches");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                resetCounts(leftDrive, rightDrive);
            }

            // Counts can be negative depending on which way a motor is mounted,
            // so use the size of the number.
            int left = Math.abs(leftDrive.getCurrentPosition());
            int right = Math.abs(rightDrive.getCurrentPosition());
            double average = (left + right) / 2.0;

            telemetry.addData("Left counts", left);
            telemetry.addData("Right counts", right);
            telemetry.addData("Counts per inch", "%.1f  (if pushed exactly %.0f in)",
                    average / PUSH_DISTANCE_INCHES, PUSH_DISTANCE_INCHES);
            telemetry.addData("Y", "zero the counts");
            telemetry.update();
        }
    }

    private void resetCounts(DcMotor leftDrive, DcMotor rightDrive) {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Back to normal mode with the motors off, so they keep counting while pushed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
