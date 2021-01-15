package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.RingLiftSubsystem;

public class RingLiftCommand extends CommandBase {

    private final RingLiftSubsystem ringLiftSubsystem;

    public RingLiftCommand(RingLiftSubsystem subsystem) {
        ringLiftSubsystem = subsystem;
        addRequirements(ringLiftSubsystem);
    }

    @Override
    public void execute() {
        ringLiftSubsystem.moveRingLift();
    }
}
