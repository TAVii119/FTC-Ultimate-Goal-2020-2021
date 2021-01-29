package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

import static java.lang.Thread.sleep;

public class FlickerSubsystem extends SubsystemBase {

    private Servo feederServo;
    private Deadline gamepadRateLimit;
    private final static int GAMEPAD_LOCKOUT = 200;

    public FlickerSubsystem(Servo servo) {
        servo = feederServo;
    }

    public void feedRings() {
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (feederServo.getPosition() == 0.0) {
            feederServo.setPosition(0.3);
            gamepadRateLimit.reset();
        } else {
            feederServo.setPosition(0.0);
            gamepadRateLimit.reset();
        }
    }
}
