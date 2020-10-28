package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
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

    public DriveSubsystem(MotorEx flMotor, MotorEx frMotor, MotorEx blMotor, MotorEx brMotor, final double diameter) {
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
        this(new MotorEx(hMap, flMotorName), new MotorEx(hMap, frMotorName), new MotorEx(hMap, blMotorName), new MotorEx(hMap, brMotorName), diameter);
    }

    public void drive(double fwd, double rot, double strf) {
        m_drive.driveRobotCentric(strf, fwd, rot);
    }

    public double getLeftEncoderVal() {
        return m_left.getPosition();
    }

    public double getRightEncoderVal() {
        return m_right.getPosition();
    }

    public double getStrafeEncoderVal() {
        return m_strafe.getPosition();
    }

    public void resetEncoders() {
        m_left.reset();
        m_right.reset();
        m_strafe.reset();
    }
}
