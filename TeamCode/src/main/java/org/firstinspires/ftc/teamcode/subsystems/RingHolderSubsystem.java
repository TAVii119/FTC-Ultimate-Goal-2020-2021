package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
This class was created by Botosan Octavian on January 14, 2021.
This is a subsystem for the ring holder mechanism that we use.
This mechanism moves the holder to the right height.
 */

public class RingHolderSubsystem extends SubsystemBase {

    private final Servo frontholderServo, backholderServo; // As you look from the back of the robot.
    private final double initPos = 0.0, finPos = 0.5;

    public RingHolderSubsystem(final HardwareMap hMap, final String name) {
        frontholderServo = hMap.get(Servo.class, name);
        backholderServo = hMap.get(Servo.class, name);
        backholderServo.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Cycle ring holder through position.
     */
    public void moveRingHolder () {
        if (frontholderServo.getPosition() == initPos) {
            frontholderServo.setPosition(finPos);
            backholderServo.setPosition(finPos);
        } else {
            frontholderServo.setPosition(initPos);
            backholderServo.setPosition(initPos);
        }
    }
}