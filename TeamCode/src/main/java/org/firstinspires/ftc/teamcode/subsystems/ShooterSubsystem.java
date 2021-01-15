package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
This class was created by Botosan Octavian on January 14, 2021.
This is a subsystem for the ring intake that we use.
 */

public class ShooterSubsystem extends SubsystemBase {

    private final Motor shooterMotorFront;
    private final Motor shooterMotorBack;
    private boolean shooterMoving = false;
    ElapsedTime timer = new ElapsedTime();

    public ShooterSubsystem(final HardwareMap hMap, final String name) {
        shooterMotorFront = hMap.get(Motor.class, name);
        shooterMotorBack = hMap.get(Motor.class, name);
        shooterMotorBack.setInverted(true);
    }

    /**
     * Start shooter.
     */

    public void startShooter() {
        if (timer.milliseconds() >= 100 && !shooterMoving) {
            shooterMotorFront.set(1.0);
            shooterMotorBack.set(1.0);
            shooterMoving = true;
            timer.reset();
        }

        if (timer.milliseconds() >= 100 && shooterMoving) {
            shooterMotorFront.set(0.0);
            shooterMotorBack.set(0.0);
            shooterMoving = false;
            timer.reset();
        }
    }
}