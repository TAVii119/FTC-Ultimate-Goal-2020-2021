package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Cu mana pe el clan.
 */

public class DriveCommand extends CommandBase {

    private final DriveSubsystem mecDrive;
    private final DoubleSupplier m_strafe, m_forward, m_turn;
    private double multiplier;

    public DriveCommand(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier turn, DoubleSupplier strafe, double mult) {
        mecDrive = subsystem;
        m_forward = forward;
        m_turn = turn;
        m_strafe = strafe;
        multiplier = mult;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        mecDrive.drive(m_forward.getAsDouble() * multiplier,
                m_turn.getAsDouble() * multiplier,
                m_strafe.getAsDouble() * multiplier);
    }
}