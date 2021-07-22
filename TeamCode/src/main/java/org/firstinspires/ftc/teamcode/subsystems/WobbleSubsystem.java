package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.opmodes.TeleOpSimple.GAMEPAD_LOCKOUT;

public class WobbleSubsystem extends SubsystemBase {

    private Motor arm;
    private Servo servo;
    private boolean grabbing = false, armDown = false;
    private Telemetry t;

    public WobbleSubsystem(Motor arm, Servo wobbleServo, Telemetry tele) {
        this.t = tele;
        this.arm = arm;
       this.servo = wobbleServo;
    }

    public void openGrabber() {
        grabbing = false;
        servo.setPosition(0.6);
    }

    public void closeGrabber() {
        grabbing = true;
        servo.setPosition(0.0);
    }

    public boolean isArmDown() { return armDown; }

    public boolean isGrabbing(){
        return grabbing;
    }

    public void driveWobbleArm(double armSpeed) {
        this.arm.set(armSpeed);
    }

    public Motor getMotor(){
        return arm;
    }
}
