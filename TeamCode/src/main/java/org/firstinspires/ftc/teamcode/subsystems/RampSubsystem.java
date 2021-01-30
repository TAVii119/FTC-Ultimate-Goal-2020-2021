package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class RampSubsystem extends SubsystemBase {

    private Servo rampServo;
    private double rampServoPos = 0;

    public RampSubsystem(Servo servo) {
        servo = rampServo; // shooter servo
    }

    public void increasePos () {
        rampServoPos += 0.01;
    }

    public void decreasePos () {
        rampServoPos -= 0.01;
    }
}
