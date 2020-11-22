package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

/**
 * TeleOpTest for team Delta Force
 * Created on 21.11.2020 by Tavi
 */

@TeleOp(name="TeleOpTest", group="Test")

public class TeleOpTest extends LinearOpMode {

    // DEFINE ROBOT HARDWARE
    HardwareTest map = new HardwareTest();

    //INSTANTIATE AND DEFINE VARIABLES
    double flPower, frPower, blPower, brPower = 0;
    private final static int GAMEPAD_LOCKOUT = 200; // PRESS DELAY IN MS
    Deadline gamepadRateLimit;

    @Override
    public void runOpMode() {
        map.init(hardwareMap);

        // Wait for the game to start
        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {


            //-//-----------\\-\\
            //-// GAMEPAD 1 \\-\\
            //-//-----------\\-\\

            // Drive
            flPower = -gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
            frPower = -gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
            blPower = -gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;
            brPower = -gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;

            map.flMotor.setPower(flPower);
            map.frMotor.setPower(frPower);
            map.blMotor.setPower(blPower);
            map.brMotor.setPower(brPower);
        }
    }
}
