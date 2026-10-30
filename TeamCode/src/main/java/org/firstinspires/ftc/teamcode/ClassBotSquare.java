package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * TEST AUTONOMOUS: drive a 24-inch square: forward 24 in, turn left 90 degrees, four times.
 * If distance and turning are both accurate, the robot ends where it started, facing the same way.
 *
 * Both driving and turning use the drive encoders, so no gyro is needed. (We tried gyro turns
 * first on 10/29-10/30, but this Control Hub's gyro never reported the robot turning.)
 *
 * How an encoder turn works: to turn in place, the left wheels roll backward and the right wheels
 * roll forward, each along a circle whose diameter is the robot's track width (the distance
 * between the left and right wheels). A full 360-degree turn rolls each side pi x track width.
 *
 * Configuration names: left_drive, right_drive (same as Class Bot Drive).
 */
@Autonomous(name = "Class Bot Square", group = "Class Bot", preselectTeleOp = "Class Bot Drive")
public class ClassBotSquare extends LinearOpMode {

    // ---- Driving (tuned on 10/29: the Class Bot lands on 24 inches). ----
    private static final double COUNTS_PER_INCH = 75.4;
    private static final double SIDE_INCHES = 24.0;
    private static final double DRIVE_POWER = 0.4;

    // ---- Turning. ----
    // "Track width": the distance between the left and right wheels, in inches, tuned by testing:
    //   turns too little (less than 90 degrees) -> make it BIGGER
    //   turns too much                         -> make it SMALLER
    //   new value = (current value) x 90 / (degrees it actually turned)
    // History: 15 (first guess) turned about 121 degrees per corner on 10/30,
    // so 15 x 90 / 121 = 11.2.
    private static final double TRACK_WIDTH_INCHES = 11.2;
    private static final double TURN_POWER = 0.3;

    // If a move hasn't finished after this long, something's wrong (like an unplugged encoder).
    private static final double MOVE_TIMEOUT_SECONDS = 5.0;

    private DcMotor leftDrive;
    private DcMotor rightDrive;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Ready", "will drive a %.0f-inch square", SIDE_INCHES);
        telemetry.addData("Tip", "line the robot up with a tile seam to judge the turns");
        telemetry.update();
        waitForStart();

        for (int side = 1; side <= 4 && opModeIsActive(); side++) {
            driveInches(SIDE_INCHES, "Side " + side + " of 4");
            turnDegrees(90, "Corner " + side + " of 4");
        }

        // Back to normal mode so Class Bot Drive works right after this.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Done. How far is the robot from its starting mark, "
                + "and is it facing the same way?");
        telemetry.update();
        while (opModeIsActive()) {
            idle();
        }
    }

    /** Drive straight a set number of inches (negative = backward). */
    private void driveInches(double inches, String step) {
        int counts = (int) Math.round(inches * COUNTS_PER_INCH);
        moveBy(counts, counts, DRIVE_POWER, step);
    }

    /** Turn in place a set number of degrees. Positive = left (counterclockwise). */
    private void turnDegrees(double degrees, String step) {
        // How far each side rolls: this turn's share of a full circle of diameter TRACK_WIDTH.
        double arcInches = Math.PI * TRACK_WIDTH_INCHES * (degrees / 360.0);
        int counts = (int) Math.round(arcInches * COUNTS_PER_INCH);
        // Turning left: left wheels backward, right wheels forward.
        moveBy(-counts, counts, TURN_POWER, step);
    }

    /** Move each side by its own number of encoder counts, then stop. */
    private void moveBy(int leftCounts, int rightCounts, double power, String step) {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setTargetPosition(leftCounts);
        rightDrive.setTargetPosition(rightCounts);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime timer = new ElapsedTime();
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        while (opModeIsActive() && timer.seconds() < MOVE_TIMEOUT_SECONDS
                && (leftDrive.isBusy() || rightDrive.isBusy())) {
            telemetry.addData("Step", step);
            telemetry.addData("Counts", "left %d / %d, right %d / %d",
                    leftDrive.getCurrentPosition(), leftCounts,
                    rightDrive.getCurrentPosition(), rightCounts);
            telemetry.update();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(200);   // a short pause so the robot settles before the next move
    }
}
