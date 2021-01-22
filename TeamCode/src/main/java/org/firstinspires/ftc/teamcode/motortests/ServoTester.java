package org.firstinspires.ftc.teamcode.motortests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*
This class was created by Botosan Octavian on January 15, 2021.
The purpose of this class is to test our servos and get the positions
to further use in our opmodes.
 */

@TeleOp(name = "ServoTester", group = "Tests")
//@Disabled
public class ServoTester extends LinearOpMode {

    // Define class members
    public Servo servo1;
    public Servo servo2;
    boolean useBothServos = false;

    @Override
    public void runOpMode() {

        // Get hardware from hub.
        servo1 = hardwareMap.get(Servo.class, "shooterServo");
        servo2 = hardwareMap.get(Servo.class, "loaderBackServo");
        servo2.setDirection(Servo.Direction.REVERSE);
        servo1.setDirection(Servo.Direction.REVERSE);

        // Wait for the start button.
        telemetry.addData(">", "Press Start to run servos.");
        telemetry.update();

        servo1.setPosition(0.0);
        servo2.setPosition(0.0);
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while (opModeIsActive()) {
            servo1.setPosition(-gamepad1.left_stick_y);
            //servo2.setPosition(-gamepad1.left_stick_y);


            // Display the current value
            telemetry.addData("Servo position: ", + servo1.getPosition());
            telemetry.addData("Servo2 position: ", + servo2.getPosition());
            telemetry.update();
        }
    }
}