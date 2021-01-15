package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

public class PickUpCommand extends CommandBase {

    private final WobbleSubsystem wobblySystem;
    private Motor motor;
    private ElapsedTime timer;

    public PickUpCommand(WobbleSubsystem subby, ElapsedTime time){
        wobblySystem = subby;
        motor = wobblySystem.getMotor();

        timer = time;

        addRequirements(subby);
    }

    @Override
    public void initialize(){
        timer.reset();
        motor.setPositionCoefficient(0.005);
        motor.setPositionTolerance(10);
        motor.setTargetPosition(-35);
        motor.stopMotor();
    }

    @Override
    public void execute(){
        wobblySystem.armUp();
    }

    @Override
    public void end(boolean interruptable){
        timer.reset();
        wobblySystem.motorStop();
    }

    @Override
    public boolean isFinished(){
        return motor.atTargetPosition() || timer.seconds() > 0.85;
    }
}
