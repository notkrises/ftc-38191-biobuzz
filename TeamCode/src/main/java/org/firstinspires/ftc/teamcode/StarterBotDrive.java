/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * TeleOp for the team's goBILDA BIOBUZZ StarterBot (tank drive).
 * Based on FIRST's BasicOpMode_Linear sample.
 *
 * Controls (gamepad 1):
 *   Left stick up/down   drive forward/back
 *   Right stick left/right  turn
 *   Right bumper (hold)  intake (motor + both front servos) collects
 *   Left bumper (hold)   intake pushes balls back out
 *   Right trigger (hold) launcher wheel spins up and stays at speed
 *   D-pad up / down      launcher faster / slower (shown on the Driver Hub)
 *   A (hold)             feeder pushes balls into the launcher, only once the wheel is at speed
 *   X (hold)             TEST ONLY: runs the feeder by itself, ignoring the launcher
 *
 * Configuration names: left_drive, right_drive, intake, launcher (goBILDA 5202/3/4 motors)
 * and left_intake_servo, right_intake_servo, windmillServo (Continuous Rotation Servos).
 */

@TeleOp(name="StarterBot Drive", group="StarterBot")
public class StarterBotDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor intake = null;
    // The two front servos with grippy wheels. They spin with the intake motor to pull balls in.
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private DcMotorEx launcher = null;
    private CRServo feeder = null;

    // Launcher speed is in encoder ticks per second. The launcher motor has 28 ticks per
    // revolution, so 1000 ticks/sec is about 2140 rpm. goBILDA's example uses 1250, which
    // was too fast for our launcher, so we start lower. The driver adjusts it with the D-pad;
    // once we find the speed that scores, put that number here.
    private static final double LAUNCHER_START_VELOCITY = 1000;
    private static final double LAUNCHER_STEP = 50;          // one D-pad press changes it by this much
    private static final double LAUNCHER_LOWEST = 300;
    private static final double LAUNCHER_HIGHEST = 2400;
    // Only feed a ball once the wheel is at least this fraction of the target speed.
    private static final double LAUNCHER_READY_FRACTION = 0.96;
    // How hard the motor works to hold its speed. P corrects for slowing down;
    // F is the steady push needed just to keep the wheel spinning.
    private static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(40, 0, 0, 12.5);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // tryGet returns null instead of crashing if "intake" isn't in the configuration,
        // so the robot can still drive before the intake is wired up.
        intake = hardwareMap.tryGet(DcMotor.class, "intake");

        // The two intake servos face opposite ways (mirror images), so one is reversed
        // to make both wheels pull the ball inward. These directions are goBILDA's.
        leftIntakeServo = hardwareMap.tryGet(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.tryGet(CRServo.class, "right_intake_servo");
        if (leftIntakeServo != null) {
            leftIntakeServo.setDirection(CRServo.Direction.FORWARD);
        }
        if (rightIntakeServo != null) {
            rightIntakeServo.setDirection(CRServo.Direction.REVERSE);
        }

        // The launcher is a DcMotorEx because it needs speed control, which plain DcMotor doesn't have.
        launcher = hardwareMap.tryGet(DcMotorEx.class, "launcher");
        if (launcher != null) {
            // RUN_USING_ENCODER: the motor reads its own speed and adjusts power to hold it.
            // This only works if the motor's encoder cable is plugged into the hub.
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
            // FLOAT lets the wheel coast to a stop instead of braking hard.
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launcher.setDirection(DcMotor.Direction.FORWARD);
        }

        // The feeder ("windmill") is a continuous rotation servo: it spins like a motor.
        feeder = hardwareMap.tryGet(CRServo.class, "windmillServo");
        if (feeder != null) {
            feeder.setDirection(CRServo.Direction.REVERSE);
        }

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // The launcher's target speed can change during the match, so it's a variable, not a constant.
        double launcherTarget = LAUNCHER_START_VELOCITY;
        // Remember whether each D-pad button was down last time through the loop, so that
        // one press changes the speed once instead of 50 times a second while it's held.
        boolean dpadUpWasDown = false;
        boolean dpadDownWasDown = false;

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Intake: hold right bumper to collect, hold left bumper to push balls back out.
            double intakePower = 0.0;
            if (gamepad1.right_bumper) {
                intakePower = 1.0;
            } else if (gamepad1.left_bumper) {
                intakePower = -1.0;
            }
            if (intake != null) {
                intake.setPower(intakePower);
            }
            if (leftIntakeServo != null) {
                leftIntakeServo.setPower(intakePower);
            }
            if (rightIntakeServo != null) {
                rightIntakeServo.setPower(intakePower);
            }

            // Launcher speed: D-pad up = faster, D-pad down = slower, one step per press.
            if (gamepad1.dpad_up && !dpadUpWasDown) {
                launcherTarget = Math.min(launcherTarget + LAUNCHER_STEP, LAUNCHER_HIGHEST);
            }
            if (gamepad1.dpad_down && !dpadDownWasDown) {
                launcherTarget = Math.max(launcherTarget - LAUNCHER_STEP, LAUNCHER_LOWEST);
            }
            dpadUpWasDown = gamepad1.dpad_up;
            dpadDownWasDown = gamepad1.dpad_down;

            // Launcher: hold right trigger to spin the wheel up and keep it at speed.
            double launcherVelocity = 0.0;
            if (launcher != null) {
                if (gamepad1.right_trigger > 0.5) {
                    launcher.setVelocity(launcherTarget);
                } else {
                    launcher.setPower(0.0);
                }
                launcherVelocity = launcher.getVelocity();
            }
            boolean launcherReady = launcherVelocity >= launcherTarget * LAUNCHER_READY_FRACTION;

            // Feeder: hold A to push balls into the launcher, but only once the wheel is
            // fast enough. Feeding a slow wheel makes the ball fall short.
            // TEST ONLY: hold X to run the feeder by itself, without waiting for the launcher.
            // Remove this before competitions so a ball can't be fed into a slow wheel.
            boolean feeding = (gamepad1.a && launcherReady) || gamepad1.x;
            if (feeder != null) {
                feeder.setPower(feeding ? 1.0 : 0.0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            // Raw stick readings. With hands off the gamepad, both should be 0.00.
            telemetry.addData("Sticks", "drive (%.2f), turn (%.2f)", drive, turn);
            telemetry.addData("Intake", intake == null ? "not in configuration" : String.format("%.2f", intakePower));
            telemetry.addData("Intake servos", "left %s, right %s",
                    leftIntakeServo == null ? "not in configuration" : "OK",
                    rightIntakeServo == null ? "not in configuration" : "OK");
            if (launcher == null) {
                telemetry.addData("Launcher", "not in configuration");
            } else {
                telemetry.addData("Launcher", "%.0f / %.0f ticks/sec  %s", launcherVelocity,
                        launcherTarget, launcherReady ? "READY" : "spinning up");
            }
            telemetry.addData("Feeder", feeder == null ? "not in configuration" : (feeding ? "feeding" : "stopped"));
            telemetry.update();
        }
    }
}
