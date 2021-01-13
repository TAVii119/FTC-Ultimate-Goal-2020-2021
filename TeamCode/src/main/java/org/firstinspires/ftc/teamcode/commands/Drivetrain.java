package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/*
This class was created by Botosan Octavian on October 28, 2020.
This class is used to control the DriveSubsystem.java class.
We take input from our Playstation DualShock 4 controller and translate in into instructions
for the robot.
 */

public class Drivetrain extends CommandBase {
    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_rotation;
    private final DoubleSupplier m_strafe;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param forward The control input for driving forwards/backwards
     * @param rotation The control input for turning
     * @param strafe The control input for strafing
     */

    public Drivetrain(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation, DoubleSupplier strafe) {
        m_drive = subsystem;
        m_forward = forward;
        m_rotation = rotation;
        m_strafe = strafe;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.drive(m_forward.getAsDouble(), m_rotation.getAsDouble(), m_strafe.getAsDouble());
    }
}
