package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RampSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

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
    public void execute() {
        localizationSystem.runLocalization();
    }

    @Override
    public void end(boolean interrupted){
        localizationSystem.stopLocalization();
    }
}
