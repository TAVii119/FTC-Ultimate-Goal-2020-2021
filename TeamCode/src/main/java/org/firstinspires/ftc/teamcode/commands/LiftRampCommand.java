package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.RampSubsystem;

public class LiftRampCommand extends CommandBase {

    private final RampSubsystem rampSubsystem;

    public LiftRampCommand(RampSubsystem subsystem) {
        rampSubsystem = subsystem;
        addRequirements(rampSubsystem);
    }

    @Override
    public void execute() {
        rampSubsystem.topGoalPos();
    }
}
