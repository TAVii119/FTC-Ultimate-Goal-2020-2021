package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

public class WobbleServoCommand extends CommandBase {

    private final WobbleSubsystem wobbleSubsystem;

    public WobbleServoCommand(WobbleSubsystem subsystem) {
        wobbleSubsystem = subsystem;
        addRequirements(wobbleSubsystem);
    }

    @Override
    public void execute() {
        wobbleSubsystem.moveWobbleServo();
    }
}
