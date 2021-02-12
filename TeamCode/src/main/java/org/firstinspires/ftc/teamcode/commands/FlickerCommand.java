package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.util.TimedAction;
import org.firstinspires.ftc.teamcode.util.Timing;
import java.util.concurrent.TimeUnit;

public class FlickerCommand extends CommandBase {

    private final FlickerSubsystem flickerSubsystem;
    private ElapsedTime runtime = new ElapsedTime();

    public FlickerCommand(FlickerSubsystem subsystem) {
        flickerSubsystem = subsystem;
        addRequirements(flickerSubsystem);
    }

    @Override
    public void initialize() {
        flickerSubsystem.flickReset();
    }

    @Override
    public void execute() {
        flickerSubsystem.flickReset();
        flickerSubsystem.flick();
    }

    @Override
    public void end(boolean interrupted) {
        flickerSubsystem.homePos();
    }

    public void returnHome() {
        flickerSubsystem.homePos();
    }

}
