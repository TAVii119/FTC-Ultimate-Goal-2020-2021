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

    public void pickMeUp() {
        servo.setPosition(0.5);
    }

    public void putMeDown() {
        servo.setPosition(0.0);
    }

    public void motorStop() {
        motor.stopMotor();
    }

    public Motor getMotor(){
        return motor;
    }

    public void armUp(){
        motor.set(0.75);
    }

    public void armDown(){
        motor.set(0.35);
    }

    public void autonDown() {
        motor.set(0.20);
    }

    public void autonUp() {
        motor.set(0.45);
    }

    @Override
    public void periodic() {
        tele.addData("Motor Position: ", motor.getCurrentPosition());
        tele.addData("Servo Position: ", servo.getPosition());
        tele.update();
    }
}
