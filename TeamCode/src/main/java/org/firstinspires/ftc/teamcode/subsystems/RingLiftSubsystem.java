package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.opmodes.TeleOpSimple.GAMEPAD_LOCKOUT;

public class RingLiftSubsystem extends SubsystemBase {

    private Servo loaderFrontServo, loaderBackServo;
    boolean up = false;

    public RingLiftSubsystem(Servo servo1, Servo servo2) {
        loaderFrontServo = servo1;
        loaderBackServo = servo2;
    }

    public void moveRingLift() {
        loaderFrontServo.setPosition(.215);
        loaderBackServo.setPosition(.215);
        up = true;
    }

    public void returnRingLift() {
        loaderFrontServo.setPosition(0);
        loaderBackServo.setPosition(0);
        up = false;
    }

    public boolean isLiftUp() {
        return up;
    }
}
