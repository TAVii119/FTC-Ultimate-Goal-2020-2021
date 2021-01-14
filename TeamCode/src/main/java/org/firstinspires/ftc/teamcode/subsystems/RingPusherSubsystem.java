package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
This class was created by Botosan Octavian on January 14, 2021.
This is a subsystem for the ring pusher that we use.
This system uses a servomotor with a 3D printed arm to push rings into our shooter.
 */

public class RingPusherSubsystem extends SubsystemBase {

    private final Servo ringPusherServo;
    private final double initPos = 0.0, finPos = 0.5;
    private boolean servoMoving = false;
    ElapsedTime timer = new ElapsedTime();

    public RingPusherSubsystem(final HardwareMap hMap, final String name) {
        ringPusherServo = hMap.get(Servo.class, name);
    }

    /**
     * Start ring intake.
     */
    public void pushRing() {
        if (timer.milliseconds() >= 100 && !servoMoving) {
            ringPusherServo.setPosition(finPos);
            ringPusherServo.setPosition(finPos);
            servoMoving = true;
            timer.reset();
        }

        if (timer.milliseconds() >= 100 && servoMoving) {
            ringPusherServo.setPosition(initPos);
            ringPusherServo.setPosition(initPos);
            servoMoving = false;
            timer.reset();
        }
    }
}