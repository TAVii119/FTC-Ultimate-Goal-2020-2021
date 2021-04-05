package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final DoubleSupplier leftY, leftX, rightX;

    public DriveCommand(DriveSubsystem drive, DoubleSupplier leftY,
                               DoubleSupplier leftX, DoubleSupplier rightX) {
        this.drive = drive;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.initialize();
    }

    @Override
    public void execute() {
        drive.execute();
        drive.drive(leftY.getAsDouble(), leftX.getAsDouble(), rightX.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
    }
}