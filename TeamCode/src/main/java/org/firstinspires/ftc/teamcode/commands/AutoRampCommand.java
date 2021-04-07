package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RampSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class AutoRampCommand extends CommandBase {

    private final RampSubsystem rampSubsystem;
    private final LocalizationSubsystem localizationSubsystem;
    private Telemetry tele;

    public AutoRampCommand(RampSubsystem subsystem, LocalizationSubsystem lSubsystem) {
        rampSubsystem = subsystem;
        localizationSubsystem = lSubsystem;
        addRequirements(rampSubsystem);
    }

    @Override
    public void execute() {
        rampSubsystem.setRampPosition(localizationSubsystem.getRampPosition());
    }
}
