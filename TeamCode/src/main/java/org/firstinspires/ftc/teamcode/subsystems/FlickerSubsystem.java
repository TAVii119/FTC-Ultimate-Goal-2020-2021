package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.TimedAction;

import java.util.concurrent.TimeUnit;

import static java.lang.Thread.sleep;

public class FlickerSubsystem extends SubsystemBase {

    private final Servo flickerServo;
    private TimedAction flickerAction;

    public FlickerSubsystem(final HardwareMap hMap, final String name) {
        flickerServo = hMap.get(Servo.class, name);
    }

    public void flick() {
        flickerServo.setPosition(0.3);
    }

    public void flickReset() {
        flickerServo.setPosition(0.0);
    }
}
