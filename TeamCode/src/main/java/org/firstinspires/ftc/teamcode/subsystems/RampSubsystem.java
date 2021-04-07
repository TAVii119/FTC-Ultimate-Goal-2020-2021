package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class RampSubsystem extends SubsystemBase {

    private Servo shooterServo;
    private double rampServoPos = 0;
    Telemetry tele;

    public RampSubsystem(Servo servo, Telemetry telemetry) {
        shooterServo = servo; // shooter servo
        tele = telemetry;
    }

    public void setRampPosition(double pos) {
        shooterServo.setPosition(pos);
    }

    public void topGoalPos () {
        shooterServo.setPosition(0.12);
    }

    public void powershotPos () {
        shooterServo.setPosition(0.08);
    }

    public void upperPos () {
        shooterServo.setPosition(0.09);
    }

    public void rampPos (double pos) {
        shooterServo.setPosition(pos);
        rampServoPos = pos;
    }

    @Override
    public void periodic() {
//        tele.addData("Ramp Position: ", shooterServo.getPosition());
//        tele.addData("Debug position: ", rampServoPos);
//        tele.update();
    }
}
