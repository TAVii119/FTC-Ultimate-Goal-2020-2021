package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class OuttakeCommand extends CommandBase {

    private final IntakeSubsystem intakeSystem;

    public OuttakeCommand(IntakeSubsystem subby) {
        intakeSystem = subby;
        addRequirements(intakeSystem);
    }

    @Override
    public void execute() {
        intakeSystem.outtake();
    }

    @Override
    public void cancel() {
        intakeSystem.stop();
        CommandScheduler.getInstance().cancel(this);
    }
}
