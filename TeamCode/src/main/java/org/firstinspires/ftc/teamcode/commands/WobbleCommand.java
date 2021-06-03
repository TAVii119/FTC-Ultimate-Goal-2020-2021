package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import java.util.function.DoubleSupplier;

public class WobbleCommand extends CommandBase {

    private final WobbleSubsystem wobbleSubsystem;
    private final DoubleSupplier armSpeed;

    public WobbleCommand(WobbleSubsystem subsystem, DoubleSupplier speed) {
        wobbleSubsystem = subsystem;
        armSpeed = speed;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
//        wobbleSubsystem.driveWobbleArm(-armSpeed.getAsDouble() * .65);
    }
}
