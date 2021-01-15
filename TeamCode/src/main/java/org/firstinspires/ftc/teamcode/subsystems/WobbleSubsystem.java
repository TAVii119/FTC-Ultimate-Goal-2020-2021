package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WobbleSubsystem extends SubsystemBase {

    Servo servo;
    Motor motor;
    Telemetry tele;

    public WobbleSubsystem(Servo pickMeUp, Motor wobbleMotor, Telemetry telemetry){
        servo = pickMeUp;
        motor = wobbleMotor;
        tele = telemetry;

        motor.resetEncoder();
    }

    public void moveWobbleMotor(double spd, double pos) {
        motor.set(spd);
        servo.setPosition(pos);
    }

    @Override
    public void periodic() {
        tele.addData("Motor Position: ", motor.getCurrentPosition());
        tele.addData("Servo Position: ", servo.getPosition());
        tele.update();
    }
}
