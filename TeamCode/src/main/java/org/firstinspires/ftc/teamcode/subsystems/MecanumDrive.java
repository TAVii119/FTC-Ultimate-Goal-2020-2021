package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class MecanumDrive extends CommandBase {
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

    public MecanumDrive(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation, DoubleSupplier strafe) {
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
