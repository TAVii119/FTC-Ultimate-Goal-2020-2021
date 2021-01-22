package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

import java.util.function.DoubleSupplier;

public class WobbleMotorCommand extends CommandBase {

    private final WobbleSubsystem wobbleSubsystem;
    private double speed;

    public WobbleMotorCommand(WobbleSubsystem subsystem, double spd) {
        wobbleSubsystem = subsystem;
        speed = spd;
        addRequirements(wobbleSubsystem);
    }

    @Override
    public void execute() {
        wobbleSubsystem.moveWobbleMotor(speed);
    }
}
