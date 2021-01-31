package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intakeSystem;

    public IntakeCommand(IntakeSubsystem subby) {
        intakeSystem = subby;
        addRequirements(intakeSystem);
    }

    @Override
    public void execute() { intakeSystem.intake(); }

    @Override
    public void cancel() {
        intakeSystem.stop();
        CommandScheduler.getInstance().cancel(this);
    }
}
