package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/**
 * TeleOpTest for team Delta Force
 * Created on 21.11.2020 by Botosan Octavian
 */

@TeleOp(name="TeleOpSimple")

public class TeleOpSimple extends LinearOpMode {

    // DEFINE ROBOT HARDWARE
    SimpleHardware map = new SimpleHardware();

    //INSTANTIATE AND DEFINE VARIABLES
    double flPower, frPower, blPower, brPower = 0;
    public final static int GAMEPAD_LOCKOUT = 200; // PRESS DELAY IN MS
    public double intakeLatch = 0.0, intakeUnlatch = 0.3;
    public double loaderPosLoad = 0.0, feederPosMid = 0.18, loaderPosShoot = 0.23;
    public double flickerInit = 0.0, flickerPush = 0.3;
    public double wobbleGrabberGrab = 0.0, wobbleGrabberUngrab = 0.2;
    public double shooterServoPos = 0.0;
    public double chassisLimiter = 1.0;
    boolean activeIntake = false;
    Deadline gamepadRateLimit;

    @Override
    public void runOpMode() {
        map.init(hardwareMap);
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
        resetServoPosition();

        // Wait for the game to start
        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            handleGamepad();

            //-//-----------\\-\\
            //-// GAMEPAD 1 \\-\\
            //-//-----------\\-\\

            // Drive
            flPower = gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
            frPower = gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
            blPower = gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;
            brPower = gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;

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
            map.wobbleMotor.setPower(gamepad2.right_trigger * 0.2 - gamepad2.left_trigger * 0.2); // limita e 500

            // Push rings into shooter
            if (gamepad2.b) {
                map.feederServo.setPosition(flickerPush);
                sleep(200);
                map.feederServo.setPosition(flickerInit);
                sleep(200);
            }

            map.shooterServo.setPosition(shooterServoPos);

            //-//-------------------\\-\\
            //-// TELEMETRY & OTHER \\-\\
            //-//-------------------\\-\\

            telemetry.addData("Shooter Speed: ", map.shooterFrontMotor.getPower());
            telemetry.addData("Shooter Servo Position: ", shooterServoPos);
            telemetry.addData("Shooter Servo Actual Position: ", map.shooterServo.getPosition());
            telemetry.addData("Ring Lift Servo Position: ", map.loaderFrontServo.getPosition());
            telemetry.addData("Wobble Motor Position: ", map.wobbleMotor.getCurrentPosition());
            telemetry.addData("Odometry Left: ", map.flMotor.getCurrentPosition());
            telemetry.addData("Odometry Right: ", map.frMotor.getCurrentPosition());
            telemetry.addData("Odometry Strafe: ", map.blMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    /*
     * handleGamepad
     *
     * Responds to a gamepad button press.  Demonstrates rate limiting for
     * button presses.  If loop() is called every 10ms and you don't rate
     * limit, then any given button press may register as multiple button presses,
     * which in this application is problematic.
     */

    public void handleGamepad() {
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad1.a && !activeIntake) { // INTAKE
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

        if (gamepad2.dpad_up) { // SHOOTER SERVO POSITION
            shooterServoPos += 0.01;
            gamepadRateLimit.reset();
        } else if (gamepad2.dpad_down && shooterServoPos >= 0.01) {
            shooterServoPos -= 0.01;
            gamepadRateLimit.reset();
        }

        if (gamepad1.x && map.loaderFrontServo.getPosition() != loaderPosLoad) { // RING LOADER
            map.loaderFrontServo.setPosition(loaderPosLoad);
            map.loaderBackServo.setPosition(loaderPosLoad);
            gamepadRateLimit.reset();
        } else if (gamepad1.x && map.loaderFrontServo.getPosition() != loaderPosShoot) {
            map.loaderFrontServo.setPosition(loaderPosShoot);
            map.loaderBackServo.setPosition(loaderPosShoot);
            gamepadRateLimit.reset();
        }

        if (gamepad1.y && chassisLimiter == 1) { // CHASSIS LIMITER
            chassisLimiter = 0.3;
            gamepadRateLimit.reset();
        } else if (gamepad1.y && chassisLimiter == 0.3) {
            chassisLimiter = 1.0;
            gamepadRateLimit.reset();
        }

        if (gamepad2.a && map.wobbleServo.getPosition() != wobbleGrabberGrab) { // WOBBLE GRABBER
            map.wobbleServo.setPosition(wobbleGrabberGrab);
            gamepadRateLimit.reset();
        } else if (gamepad2.a && map.wobbleServo.getPosition() != wobbleGrabberUngrab) {
            map.wobbleServo.setPosition(wobbleGrabberUngrab);
            gamepadRateLimit.reset();
        }
    }

    public void resetServoPosition() {
        map.wobbleServo.setPosition(0.0);
        map.loaderFrontServo.setPosition(0.0);
        map.loaderBackServo.setPosition(0.0);
        map.feederServo.setPosition(0.0);
        map.intakeServo.setPosition(0.0);
        map.shooterServo.setPosition(0.0);
    }
}