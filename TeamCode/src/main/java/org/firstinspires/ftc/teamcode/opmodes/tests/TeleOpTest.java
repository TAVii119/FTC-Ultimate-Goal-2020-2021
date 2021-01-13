package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/**
 * TeleOpTest for team Delta Force
 * Created on 21.11.2020 by Botosan Octavian
 */

@TeleOp(name="TeleOpTest", group="Test")

public class TeleOpTest extends LinearOpMode {

    // DEFINE ROBOT HARDWARE
    HardwareTest map = new HardwareTest();

    //INSTANTIATE AND DEFINE VARIABLES
    double flPower, frPower, blPower, brPower = 0;
    public final static int GAMEPAD_LOCKOUT = 200; // PRESS DELAY IN MS
    public double intakeLatch = 0.5, intakeUnlatch = 0.0;;
    public double feederPosLoad = 0.0, feederPosMid = 0.5, feederPosShoot = 0.7;
    public double wobbleGrabberGrab = 0.5, wobbleGrabberUngrab = 0.0;
    public double shooterServoPos = 0.0;
    public double chassisLimiter = 1.0;
    boolean activeIntake = false;
    Deadline gamepadRateLimit;


    @Override
    public void runOpMode() {
        map.init(hardwareMap);
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

        // Wait for the game to start
        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            handleGamepad();

            //-//-----------\\-\\
            //-// GAMEPAD 1 \\-\\
            //-//-----------\\-\\

            // Drive
            flPower = -gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
            frPower = -gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
            blPower = -gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;
            brPower = -gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;

            map.flMotor.setPower(flPower * chassisLimiter);
            map.frMotor.setPower(frPower * chassisLimiter);
            map.blMotor.setPower(blPower * chassisLimiter);
            map.brMotor.setPower(brPower * chassisLimiter);

            // Shooter Power
            map.shooterFrontMotor.setPower(gamepad1.right_trigger);
            map.shooterBackMotor.setPower(gamepad1.right_trigger);

            //-//-----------\\-\\
            //-// GAMEPAD 2 \\-\\
            //-//-----------\\-\\

            // Handle Wobble Mechanism
            map.wobbleMotor.setPower(gamepad2.right_trigger * 0.1 - gamepad2.left_trigger * 0.1);
            if (gamepad2.a) {
                map.wobbleGrabberServo.setPosition(wobbleGrabberGrab);
            } else if (gamepad2.b) {
                map.wobbleGrabberServo.setPosition(wobbleGrabberUngrab);
            }

            //-//-------------------\\-\\
            //-// TELEMETRY & OTHER \\-\\
            //-//-------------------\\-\\

            telemetry.addData("Shooter Speed: ", map.shooterFrontMotor.getPower());
            telemetry.addData("Shooter Servo Position: ", map.shooterServo.getPosition());
            telemetry.addData("Loader Servo Position: ", map.loaderFrontServo.getPosition());
            telemetry.update();
        }
    }

    /*
     * handleGamepad
     *
     * Responds to a gamepad button press.  Demonstrates rate limiting for
     * button presses.  If loop() is called every 10ms and and you don't rate
     * limit, then any given button press may register as multiple button presses,
     * which in this application is problematic.
     */

    public void handleGamepad() {
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad1.a && !activeIntake) {
            map.intakeMotor.setPower(1);
            activeIntake = true;
            gamepadRateLimit.reset();
        } else if (gamepad1.a && activeIntake) {
            map.intakeMotor.setPower(0);
            activeIntake = false;
            gamepadRateLimit.reset();
        } else if (gamepad1.b && !activeIntake) {
            map.intakeMotor.setPower(-1);
            activeIntake = true;
            gamepadRateLimit.reset();
        } else if (gamepad1.b && activeIntake) {
            map.intakeMotor.setPower(0);
            activeIntake = false;
            gamepadRateLimit.reset();
        }

        if (gamepad1.dpad_up) {
            shooterServoPos += 0.25;
            gamepadRateLimit.reset();
        } else if (gamepad1.dpad_down && shooterServoPos >= 0.25) {
            shooterServoPos -= 0.25;
            gamepadRateLimit.reset();
        }

        if (gamepad1.x && map.loaderFrontServo.getPosition() == feederPosLoad) {
            map.loaderFrontServo.setPosition(feederPosMid);
            map.loaderBackServo.setPosition(feederPosMid);
            gamepadRateLimit.reset();
        } else if (gamepad1.x && map.loaderFrontServo.getPosition() == feederPosMid) {
            map.loaderFrontServo.setPosition(feederPosShoot);
            map.loaderBackServo.setPosition(feederPosShoot);
            gamepadRateLimit.reset();
        } else if (gamepad1.x && map.loaderFrontServo.getPosition() == feederPosShoot) {
            map.loaderFrontServo.setPosition(feederPosLoad);
            map.loaderBackServo.setPosition(feederPosLoad);
            gamepadRateLimit.reset();
        }

        if (gamepad1.y && chassisLimiter == 1) {
            chassisLimiter = 0.1;
            gamepadRateLimit.reset();
        } else if (gamepad1.y && chassisLimiter == 0.1) {
            chassisLimiter = 1.0;
            gamepadRateLimit.reset();
        }
    }
}