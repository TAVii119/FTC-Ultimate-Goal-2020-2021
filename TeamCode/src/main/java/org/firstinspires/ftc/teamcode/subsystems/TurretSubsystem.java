package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class TurretSubsystem extends SubsystemBase {

    private Servo turretServo;

    Telemetry tele;

    public TurretSubsystem(Servo servo, Telemetry telemetry) {
        turretServo = servo;
        tele = telemetry;
    }

    public void setTurretPos (double turretPos) {
        turretServo.setPosition(turretPos);
    }

    @Override
    public void periodic() {
//        tele.addData("Ramp Position: ", shooterServo.getPosition());
//        tele.addData("Debug position: ", rampServoPos);
//        tele.update();
    }
}
