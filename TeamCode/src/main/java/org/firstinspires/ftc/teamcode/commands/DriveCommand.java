package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Cu mana pe el clan.
 */
public class DriveCommand extends CommandBase {
    private final DriveSubsystem mecDrive;
    private final DoubleSupplier m_strafe;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_turn;

    public DriveCommand(DriveSubsystem subsystem, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn){
        mecDrive = subsystem;
        m_strafe = strafe;
        m_forward = forward;
        m_turn = turn;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        mecDrive.drive(m_strafe.getAsDouble(), m_forward.getAsDouble(), m_turn.getAsDouble());
    }
}