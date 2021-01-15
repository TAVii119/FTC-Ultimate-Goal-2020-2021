package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleSupplier;

/*
This class was created by Botosan Octavian on October 28, 2020.
This is a subsystem for the Mecanum drivetrain that we use.
Mecanum drivetrains are holonomic (they can move in any direction and also rotate in place).
 */

public class DriveSubsystem extends SubsystemBase {

    private final Motor flMotor, frMotor, blMotor, brMotor;
    private boolean isLimited = false;
    private double driveLimiter;

    public DriveSubsystem(final HardwareMap hMap, final String name) {
        flMotor = new MotorEx(hMap, "flMotor", MotorEx.GoBILDA.RPM_435);
        frMotor = new MotorEx(hMap, "frMotor", MotorEx.GoBILDA.RPM_435);
        blMotor = new MotorEx(hMap, "blMotor", MotorEx.GoBILDA.RPM_435);
        brMotor = new MotorEx(hMap, "brMotor", MotorEx.GoBILDA.RPM_435);

        frMotor.setInverted(true);
        brMotor.setInverted(true);
    }

    /**
     * Drive chassis.
     */

    public void drive(double forward, double rotation, double strafe) {
        flMotor.set(forward * getlimiter());
        frMotor.set(forward * getlimiter());
        blMotor.set(forward * getlimiter());
        brMotor.set(forward * getlimiter());
    }

    public double getlimiter() {
        if (isLimited = false) {
            isLimited = true;
            driveLimiter = 0.1;
        }else if (isLimited = true) {
            isLimited = false;
            driveLimiter = 1.0;
        }
        return driveLimiter;
    }
}
