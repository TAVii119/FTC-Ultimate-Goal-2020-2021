package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Cu mana pe el clan.
 */
public class DriveCommand extends CommandBase {

    // The subsystem the command runs on
    private final DriveSubsystem driveSubsystem;

    public DriveCommand(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}