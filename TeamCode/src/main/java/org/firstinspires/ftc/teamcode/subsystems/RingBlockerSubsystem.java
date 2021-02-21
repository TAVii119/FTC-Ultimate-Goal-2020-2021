package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class RingBlockerSubsystem extends SubsystemBase {

    private Servo ringBlockerLeft, ringBlockerRight;
    boolean down = false;

    public RingBlockerSubsystem(Servo servo1, Servo servo2) {
        ringBlockerLeft = servo1;
        ringBlockerRight = servo2;
    }

    public void blockRings() {
        ringBlockerLeft.setPosition(.3);
        ringBlockerRight.setPosition(.3);
        down = true;
    }

    public void unBlockRings() {
        ringBlockerLeft.setPosition(0.01);
        ringBlockerRight.setPosition(0.01);
        down = false;
    }

    public boolean isBlockerDown() {
        return down;
    }
}
