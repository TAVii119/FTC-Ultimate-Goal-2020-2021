package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

public class PutDownCommand extends CommandBase {

    private final WobbleSubsystem wobblySystem;

    private Motor motor;
    private ElapsedTime timer;

    public PutDownCommand(WobbleSubsystem subby, ElapsedTime timey) {
        wobblySystem = subby;
        motor = wobblySystem.getMotor();



        timer = timey;

        addRequirements(subby);
    }

    @Override
    public void initialize() {
        timer.reset();
        motor.setPositionCoefficient(0.005);
        motor.setPositionTolerance(10);
        motor.setTargetPosition(-400);
        motor.stopMotor();

    }

    @Override
    public void execute() {
        wobblySystem.armDown();
    }

    @Override
    public void end(boolean interruptable) {
        timer.reset();
        wobblySystem.motorStop();
    }

    @Override
    public boolean isFinished() {
        return motor.atTargetPosition() || timer.seconds() > 2.0;
    }
}
