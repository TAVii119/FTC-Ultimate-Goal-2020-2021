package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.util.TimedAction;

public class FlickReturnCommand extends CommandBase {

    private final FlickerSubsystem flickerSubsystem;
    private TimedAction flickerAction;

    public FlickReturnCommand(FlickerSubsystem subsystem) {
        flickerSubsystem = subsystem;
        addRequirements(flickerSubsystem);
    }

    @Override
    public void execute() {
        flickerSubsystem.flickReset();
    }
}
