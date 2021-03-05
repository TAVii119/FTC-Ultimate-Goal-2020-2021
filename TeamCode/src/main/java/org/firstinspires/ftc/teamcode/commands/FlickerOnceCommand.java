package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.util.TimedAction;
import org.firstinspires.ftc.teamcode.util.Timing;
import java.util.concurrent.TimeUnit;

public class FlickerOnceCommand extends CommandBase {

    private final FlickerSubsystem flickerSubsystem;
    private ElapsedTime runtime = new ElapsedTime();

    public FlickerOnceCommand(FlickerSubsystem subsystem) {
        flickerSubsystem = subsystem;
        addRequirements(flickerSubsystem);
    }

    @Override
    public void execute() {
        flickerSubsystem.flickOnce();
    }

    @Override
    public void end(boolean interrupted) {
        flickerSubsystem.homePos();
    }
}
