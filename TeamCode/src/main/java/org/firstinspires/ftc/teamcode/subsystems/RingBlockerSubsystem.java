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
        ringBlockerLeft.setPosition(0.44);
        ringBlockerRight.setPosition(0.41);
        down = true;
    }

    public void unBlockRings() {
        ringBlockerLeft.setPosition(0.05);
        ringBlockerRight.setPosition(0.05);
        down = false;
    }

    public void initBlockRings() {
        ringBlockerLeft.setPosition(0.04);
        ringBlockerRight.setPosition(0.04);
        down = false;
    }


    public boolean isBlockerDown() {
        return down;
    }
}
