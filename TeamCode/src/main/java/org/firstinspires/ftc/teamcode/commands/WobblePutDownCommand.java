package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

public class WobblePutDownCommand extends CommandBase {

    private Motor arm;
    private WobbleSubsystem wobbleSystem;

    public WobblePutDownCommand(WobbleSubsystem subsystem) {
        wobbleSystem = subsystem;
        arm = wobbleSystem.getMotor();

        addRequirements(wobbleSystem);
    }

    @Override
    public void initialize() {
        arm.setPositionCoefficient(0.005);
        arm.setPositionTolerance(10);
        arm.setTargetPosition(500);
    }

    @Override
    public void execute() {
        wobbleSystem.armDown();
    }

    @Override
    public void end(boolean interrupted) {
        wobbleSystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return arm.atTargetPosition();
    }
}
