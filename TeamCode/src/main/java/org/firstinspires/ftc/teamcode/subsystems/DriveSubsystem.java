package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
This class was created by Botosan Octavian on October 28, 2020.
This is a subsystem for the Mecanum drivetrain that we use.
Mecanum drivetrains are holonomic (they can move in any direction and also rotate in place).
 */

public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive m_drive;

    private final Encoder m_left, m_right, m_strafe;

    private final double WHEEL_DIAMETER;

    /**
     * Creates a new DriveSubsystem
     */

    public DriveSubsystem(Motor flMotor, Motor frMotor, Motor blMotor, Motor brMotor, final double diameter) {
        m_left = flMotor.encoder;
        m_right = frMotor.encoder;
        m_strafe = blMotor.encoder;

        WHEEL_DIAMETER = diameter;

        m_drive = new MecanumDrive(flMotor, frMotor, blMotor, brMotor);
    }

    /**
     * Creates a new DriveSubsystem with the hardware map and configuration names.
     */
    public DriveSubsystem(HardwareMap hMap, final String flMotorName, String frMotorName, String blMotorName, String brMotorName, final double diameter) {
        this(new Motor(hMap, flMotorName, Motor.GoBILDA.RPM_435), new Motor(hMap, frMotorName, Motor.GoBILDA.RPM_435),
                new Motor(hMap, blMotorName, Motor.GoBILDA.RPM_435), new Motor(hMap, brMotorName ,Motor.GoBILDA.RPM_435), diameter);
    }

    public void drive(double fwd, double rot, double strf) {
        m_drive.driveRobotCentric(strf, fwd, rot);
    }

    public double getLeftEncoderVal() {
        return m_left.getPosition();
    }

    public double getLeftEncoderDistance() { return m_left.getRevolutions() * WHEEL_DIAMETER * Math.PI; }

    public double getRightEncoderVal() {
        return m_right.getPosition();
    }

    public double getRightEncoderDistance() { return m_right.getRevolutions() * WHEEL_DIAMETER * Math.PI; }

    public double getStrafeEncoderVal() {
        return m_strafe.getPosition();
    }

    public double getStrafeEncoderDistance() { return m_strafe.getRevolutions() * WHEEL_DIAMETER * Math.PI; }

    public void resetEncoders() {
        m_left.reset();
        m_right.reset();
        m_strafe.reset();
    }

    public double getAverageEncoderDistance() {
        return (getLeftEncoderDistance() + getRightEncoderDistance() + getStrafeEncoderDistance()) / 3.0;
    }
}
