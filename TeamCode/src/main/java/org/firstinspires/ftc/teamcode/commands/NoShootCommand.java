package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class NoShootCommand extends CommandBase {

    private final ShooterSubsystem shooterSystem;

    public NoShootCommand(ShooterSubsystem subby){
        shooterSystem = subby;
        addRequirements(shooterSystem);
    }
    @Override
    public void execute(){
        shooterSystem.stop();
    }
}
