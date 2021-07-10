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
    private TimedAction timedAction;

    public FlickerSubsystem(Servo flickerServo, TimedAction timedAction) {
        this.flickerServo = flickerServo;
        this.timedAction = timedAction;
    }

    public boolean isRunning() {
        return timedAction.running();
    }

    public void flick() {
//        ringBlockerLeft.setPosition(0.405);
//        ringBlockerRight.setPosition(0.37);
//        RingBlockerSubsystem.down = true;
        timedAction.run();
    }
    public void flickOnce() {
        flickerServo.setPosition(0.15);
        flickerServo.setPosition(0);
    }

    public void flickReset() {
        if(!timedAction.running())
            timedAction.reset();
    }

    public void homePos() {
        flickerServo.setPosition(0);
    }
}
