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
    public void execute() {
        while (runtime.milliseconds() < 100)
            flickerSubsystem.flick();
        flickerSubsystem.flickReset();
        runtime.reset();
    }

    /*
    public void end() {
        flickerSubsystem.flickReset();
    }
     */
}
