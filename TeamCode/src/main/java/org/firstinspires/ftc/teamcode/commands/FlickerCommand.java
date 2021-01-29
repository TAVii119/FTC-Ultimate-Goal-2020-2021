package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RingLiftSubsystem;

public class FlickerCommand extends CommandBase {

    private final FlickerSubsystem flickerSubsystem;

    public FlickerCommand(FlickerSubsystem subsystem) {
        flickerSubsystem = subsystem;
        addRequirements(flickerSubsystem);
    }

    @Override
    public void execute() { flickerSubsystem.feedRings(); }
}
