package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RampSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase {

    private final TurretSubsystem turretSubsystem;
    private final DriveSubsystem driveSubsystem;
    private Telemetry tele;

    public TurretCommand(TurretSubsystem subsystem, DriveSubsystem dSubsystem) {
        turretSubsystem = subsystem;
        driveSubsystem = dSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        if(turretSubsystem.getAlignedToPowershots() == false) {
            turretSubsystem.setTurretPos(driveSubsystem.setTurretPosition());
        } else {
            turretSubsystem.setTurretPos(0.23);
        }
    }
}
