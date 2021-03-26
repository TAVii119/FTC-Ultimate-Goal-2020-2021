package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/*
 * TeleOpTest for team Delta Force
 * Created on 21.11.2020 by Botosan Octavian
*/

@TeleOp(name="TeleOpSimple")

public class TeleOpSimple extends LinearOpMode {

    // DEFINE ROBOT HARDWARE
    SimpleHardware map = new SimpleHardware();

    // INSTANTIATE AND DEFINE VARIABLES
    double flPower, frPower, blPower, brPower = 0;
    public final static int GAMEPAD_LOCKOUT = 200; // PRESS DELAY IN MS
    public double feederInit = 0.0, feederPush = 0.165;
    public double shooterServoPos = 0.0;
    public double chassisLimiter = 1.0;
    public double turretServoPos = 0;
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
            flPower = -gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
            frPower = -gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
            blPower = -gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;
            brPower = -gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;

            map.flMotor.setPower(flPower * chassisLimiter);
            map.frMotor.setPower(frPower * chassisLimiter);
            map.blMotor.setPower(blPower * chassisLimiter);
            map.brMotor.setPower(brPower * chassisLimiter);

            map.intakeMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            // Push rings into shooter
            if (gamepad1.b) {
                map.feederServo.setPosition(feederPush);
                sleep(150);
                map.feederServo.setPosition(feederInit);
                sleep(200);
            }

            map.turretServo.setPosition(turretServoPos);
            map.shooterServo.setPosition(shooterServoPos);

            //-//-----------\\-\\
            //-// GAMEPAD 2 \\-\\
            //-//-----------\\-\\


            //-//-------------------\\-\\
            //-// TELEMETRY & OTHER \\-\\
            //-//-------------------\\-\\

            telemetry.addData("Intake Speed: ", map.intakeMotor.getPower());
            telemetry.addData("Shooter Speed: ", map.shooterFrontMotor.getPower());
            telemetry.addData("Shooter Servo Position: ", shooterServoPos);
            telemetry.addData("Turret Position: ", turretServoPos);
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

        if (gamepad1.a && map.shooterFrontMotor.getPower() == 0) {
            map.shooterFrontMotor.setPower(0.7);
            map.shooterBackMotor.setPower(0.7);
            gamepadRateLimit.reset();
        } else if (gamepad1.a && map.shooterFrontMotor.getPower() > 0.2) {
            map.shooterFrontMotor.setPower(0);
            map.shooterBackMotor.setPower(0);
            gamepadRateLimit.reset();
        }

        if (gamepad1.dpad_up) { // SHOOTER SERVO POSITION
            shooterServoPos += 0.01;
            gamepadRateLimit.reset();
        } else if (gamepad1.dpad_down && shooterServoPos >= 0.01) {
            shooterServoPos -= 0.01;
            gamepadRateLimit.reset();
        }

        if (gamepad1.dpad_left) { // SHOOTER SERVO POSITION
            if (turretServoPos <= 0.37) {
                turretServoPos += 0.01;
            } else {
                turretServoPos = 0.37;
            }
            gamepadRateLimit.reset();
        } else if (gamepad1.dpad_right && turretServoPos >= 0.01) {
            turretServoPos -= 0.01;
            gamepadRateLimit.reset();
        }

        if (gamepad1.x && chassisLimiter == 1) { // CHASSIS LIMITER
            chassisLimiter = 0.3;
            gamepadRateLimit.reset();
        } else if (gamepad1.x && chassisLimiter == 0.3) {
            chassisLimiter = 1.0;
            gamepadRateLimit.reset();
        }

        if (gamepad1.y) {
            map.intakeServo.setPosition(0.6);
        }

        if (gamepad1.x) {
            map.intakeServo.setPosition(0.0);
        }

        if (gamepad2.y && map.wobbleServo.getPosition() == 0) {
            map.wobbleServo.setPosition(0.32);
        } else if (gamepad2.y && map.wobbleServo.getPosition() != 0) {
            map.wobbleServo.setPosition(0.0);
        }

        if (gamepad2.b && map.wobbleServoGrabber.getPosition() == 0) {
            map.wobbleServoGrabber.setPosition(0.49);
        } else if (gamepad2.b && map.wobbleServoGrabber.getPosition() != 0) {
            map.wobbleServoGrabber.setPosition(0.0);
        }
    }

    public void resetServoPosition() {
        map.feederServo.setPosition(0.0);
        map.intakeServo.setPosition(0.0);
        map.shooterServo.setPosition(0.0);
        map.ringBlockerLeft.setPosition(0.031);
        map.ringBlockerRight.setPosition(0.031);
        map.wobbleServo.setPosition(0.0);
        map.wobbleServoGrabber.setPosition(0.0);
        map.turretServo.setPosition(0.0);
    }
}