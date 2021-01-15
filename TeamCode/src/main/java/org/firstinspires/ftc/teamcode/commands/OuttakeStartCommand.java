package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class OuttakeStartCommand extends CommandBase {

    private final IntakeSubsystem intakeSystem;

    public OuttakeStartCommand(IntakeSubsystem subby){
        intakeSystem = subby;
        addRequirements(intakeSystem);
    }
    @Override
    public void execute(){
        intakeSystem.suck();
    }
    @Override
    public void cancel() {
        intakeSystem.stop();
        CommandScheduler.getInstance().cancel(this);
    }
}
