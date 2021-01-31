package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.opmodes.TeleOpSimple.GAMEPAD_LOCKOUT;

public class WobbleSubsystem extends SubsystemBase {

    private Motor arm;
    private Servo grabber;
    private boolean grabbing = false;

    public WobbleSubsystem(Motor arm, Servo grabber){
        this.arm = arm;
        this.grabber = grabber;

    }

    public void openGrabber(){
        grabbing = false;
        grabber.setPosition(0);
    }

    public void closeGrabber(){
        grabbing = true;
        grabber.setPosition(.2);
    }

    public boolean isGrabbing(){
        return grabbing;
    }

    public Motor getMotor(){
        return arm;
    }

    public void driveWobbleArm(double armSpeed) {
        arm.set(armSpeed);
    }
}
