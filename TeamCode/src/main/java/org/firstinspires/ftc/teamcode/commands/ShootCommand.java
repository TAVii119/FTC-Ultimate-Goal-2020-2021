package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {

    private final ShooterSubsystem shooterSystem;

    public ShootCommand(ShooterSubsystem subby) {
        shooterSystem = subby;
        addRequirements(subby);
    }

    @Override
    public void execute() {
        shooterSystem.shoot();
    }

    @Override
    public void cancel() {
        shooterSystem.stop();
        CommandScheduler.getInstance().cancel(this);
    }
}
