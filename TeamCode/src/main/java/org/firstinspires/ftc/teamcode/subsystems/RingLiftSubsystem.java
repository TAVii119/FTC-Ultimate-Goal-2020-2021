package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.opmodes.TeleOpSimple.GAMEPAD_LOCKOUT;

public class RingLiftSubsystem extends SubsystemBase {

    private Servo loaderFrontServo, loaderBackServo;
    private Deadline gamepadRateLimit;
    private final static int GAMEPAD_LOCKOUT = 200;

    public RingLiftSubsystem(Servo servo1, Servo servo2) {
        servo1 = loaderFrontServo;
        servo2 = loaderBackServo;
    }

    public void moveRingLift() {
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (loaderFrontServo.getPosition() == 0.0) {
            loaderFrontServo.setPosition(0.5);
            loaderBackServo.setPosition(0.5);
            gamepadRateLimit.reset();
        } else {
            loaderFrontServo.setPosition(0.0);
            loaderBackServo.setPosition(0.0);
            gamepadRateLimit.reset();
        }
    }
}
