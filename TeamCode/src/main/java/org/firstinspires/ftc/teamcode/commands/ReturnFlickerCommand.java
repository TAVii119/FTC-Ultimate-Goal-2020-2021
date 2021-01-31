package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.FlickerSubsystem;

public class ReturnFlickerCommand extends CommandBase {

    private final FlickerSubsystem flickerSubsystem;
    private ElapsedTime runtime = new ElapsedTime();

    public ReturnFlickerCommand(FlickerSubsystem subsystem) {
        flickerSubsystem = subsystem;
        addRequirements(flickerSubsystem);
    }

    @Override
    public void execute() {
        flickerSubsystem.flickReset();
    }
}
