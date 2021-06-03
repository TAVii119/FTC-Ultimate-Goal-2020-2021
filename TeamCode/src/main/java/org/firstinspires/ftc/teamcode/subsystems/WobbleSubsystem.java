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
    private Servo grabberLeft, grabberRight;
    private boolean grabbing = false, armDown = false;
    private Telemetry t;

    public WobbleSubsystem(Motor arm, Servo grabberLeft, Servo grabberRight, Telemetry tele) {
        this.t = tele;
        this.arm = arm;
        arm.setRunMode(Motor.RunMode.PositionControl);
        arm.setPositionCoefficient(0.1);
        arm.resetEncoder();
        arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        double kP = arm.getPositionCoefficient();
        this.grabberLeft = grabberLeft;
        this.grabberRight = grabberRight;
    }

    public void openGrabber() {
        grabbing = false;
        grabberLeft.setPosition(0.48);
        grabberRight.setPosition(0.48);
    }

    public void closeGrabber() {
        grabbing = true;
        grabberLeft.setPosition(0);
        grabberRight.setPosition(0);
    }

    public void moveMotorArm() {
        arm.setTargetPosition(118);
        arm.set(0);
        arm.setPositionTolerance(20);

        while (!arm.atTargetPosition()) {
//            t.addData("Motor pos", arm.getCurrentPosition());
//            t.update();
            arm.set(0.15);
        }

        arm.stopMotor();
        armDown = true;
    }

    public void liftMotorArm() {
        arm.setTargetPosition(75);
        arm.set(0);
        arm.setPositionTolerance(20);

        while (!arm.atTargetPosition()) {
            arm.set(0.25);
        }

        arm.stopMotor();
        arm.set(-0.16);
        armDown = false;
    }

    public boolean isArmDown() { return armDown; }

    public boolean isGrabbing(){
        return grabbing;
    }

    public Motor getMotor(){
        return arm;
    }
}
