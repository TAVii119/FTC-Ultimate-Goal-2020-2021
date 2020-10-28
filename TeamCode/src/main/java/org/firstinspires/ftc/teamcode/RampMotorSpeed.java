package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Concept: Ramp Motor Speed", group = "Concept")
//@Disabled
public class RampMotorSpeed extends LinearOpMode {

    // Define class members
    public DcMotor motor;
    public DcMotor shootMotor;
    public DcMotor shootDriver;
    public double power = 0;
    public double shootPower = 0;
    public double driverPower = 0;


    @Override
    public void runOpMode() {

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        motor = hardwareMap.get(DcMotor.class, "left_drive");
        shootMotor = hardwareMap.get(DcMotor.class, "shoot_drive");
        shootDriver = hardwareMap.get(DcMotor.class, "driver_drive");

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while (opModeIsActive()) {

            power = -1.0;
            driverPower = -1.0;
            shootPower = -1.0;


            // Display the current value
            telemetry.addData("Motor Power", "%5.2f", power);
            telemetry.addData("ShootMotor Power", "%5.2f", shootPower);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the motor to the new power and pause;
            motor.setPower(power);
            shootMotor.setPower(shootPower);
            shootDriver.setPower(driverPower);

        }

        // Turn off motor and signal done;
        motor.setPower(0);
        shootMotor.setPower(0);
        shootDriver.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
