package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

/*
 * TEST PROGRAM: the Class Bot uses its camera to find a Pollen ball and drive up to it.
 * The robot only moves while you HOLD A. Let go and it stops.
 *
 * How it works, about 20 times a second:
 *   1. Find the biggest round yellow blob in the camera image (same settings as Pollen Finder).
 *   2. Steer: if the ball is left of the image center, turn left; if right, turn right.
 *      The farther off center, the harder it turns.
 *   3. Drive: if the ball still looks small (far away), drive forward; once it looks big
 *      enough (about 1 foot away), stop.
 *
 * The webcam must be mounted on the FRONT of the robot, facing forward, right side up.
 * Configuration names: left_drive, right_drive, Webcam 1.
 */
@TeleOp(name = "Drive to Pollen (hold A)", group = "Class Bot")
public class ClassBotDriveToPollen extends LinearOpMode {

    // ---- Steering. ----
    private static final double TURN_GAIN = 0.004;   // turn power per pixel off center
    private static final double MAX_TURN = 0.3;
    private static final int CENTERED_PX = 15;       // within this many pixels counts as centered

    // ---- Driving. ----
    private static final double FORWARD_POWER = 0.3;
    // Measured 11/02: radius is about 40 px at 1 ft and 16 px at 3 ft, so stop at 40 px (about 1 ft).
    private static final int STOP_RADIUS_PX = 40;
    // From the same measurements: distance in feet is roughly 44 / radius.
    private static final double DISTANCE_FACTOR = 44.0;

    @Override
    public void runOpMode() {
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        // Plain power mode, set on purpose: a motor's mode stays on the robot between programs.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Same Pollen detection settings as the Pollen Finder program.
        ColorBlobLocatorProcessor pollenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(PollenFinder.POLLEN_COLOR)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(pollenLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.addData("Ready", "press START, then HOLD A to chase the ball");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // 1. Find the biggest round yellow blob.
            List<ColorBlobLocatorProcessor.Blob> blobs = pollenLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, blobs);
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    PollenFinder.MIN_CIRCULARITY, 1, blobs);
            ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

            double leftPower = 0;
            double rightPower = 0;

            if (blobs.isEmpty()) {
                telemetry.addData("Pollen", "none in view");
            } else {
                Circle ball = blobs.get(0).getCircle();
                int offset = (int) ball.getX() - PollenFinder.IMAGE_CENTER_X;
                int radius = (int) ball.getRadius();
                boolean centered = Math.abs(offset) < CENTERED_PX;
                boolean closeEnough = radius >= STOP_RADIUS_PX;

                // 2. Steer: positive offset means the ball is to the right, so turn right.
                double turn = Range.clip(offset * TURN_GAIN, -MAX_TURN, MAX_TURN);
                // 3. Drive forward until the ball looks big enough.
                double forward = closeEnough ? 0 : FORWARD_POWER;

                if (gamepad1.a) {
                    leftPower = forward + turn;
                    rightPower = forward - turn;
                }

                String status;
                if (closeEnough && centered) {
                    status = "ARRIVED";
                } else if (!gamepad1.a) {
                    status = "hold A to chase";
                } else {
                    status = "chasing";
                }
                telemetry.addData("Pollen", "%s (x offset %+d px)",
                        centered ? "CENTERED" : (offset < 0 ? "LEFT" : "RIGHT"), offset);
                telemetry.addData("Distance", "about %.1f ft (radius %d px, stops at %d)",
                        DISTANCE_FACTOR / Math.max(radius, 1), radius, STOP_RADIUS_PX);
                telemetry.addData("Status", status);
            }

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            telemetry.addData("Motors", "left %.2f, right %.2f", leftPower, rightPower);
            telemetry.update();
        }
        portal.close();
    }
}
