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

    public void topGoalPos () {
        shooterServo.setPosition(0.04);
    }

    public void powershotPos () {
        shooterServo.setPosition(0.02);
    }

    @Override
    public void periodic() {
        tele.addData("Ramp Position: ", shooterServo.getPosition());
        tele.update();
    }
}