package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;

public class LocalizationCommand extends CommandBase {

    private LocalizationSubsystem localizationSystem = null;

    public LocalizationCommand(LocalizationSubsystem subsystem) {
        localizationSystem = subsystem;
        addRequirements(localizationSystem);
    }

    @Override
    public void initialize() {
        localizationSystem.initialize();
    }

    @Override
    public void execute() { localizationSystem.loop(); }

    @Override
    public void end(boolean interrupted){
        localizationSystem.stopLocalization();
    }
}
