package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class RingBlockerSubsystem extends SubsystemBase {

    private Servo ringBlockerLeft, ringBlockerRight;
    boolean down;

    public RingBlockerSubsystem(Servo servo1, Servo servo2) {
        ringBlockerLeft = servo1;
        ringBlockerRight = servo2;
    }
    // 0.3 e pozitia de unblock

    public void init() {
        ringBlockerLeft.setPosition(0.3);
        ringBlockerRight.setPosition(0.3);
        down = false;
    }

    public void blockRings() {
        ringBlockerLeft.setPosition(0.65);
        ringBlockerRight.setPosition(0.65);
        down = true;
    }

    public void unBlockRings() {
        ringBlockerLeft.setPosition(0.3);
        ringBlockerRight.setPosition(0.3);
        down = false;
    }

    public boolean isBlockerDown() {
        return down;
    }
}
