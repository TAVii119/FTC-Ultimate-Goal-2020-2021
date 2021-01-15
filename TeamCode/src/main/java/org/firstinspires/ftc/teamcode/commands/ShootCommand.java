package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {

    private ShooterSubsystem shooterSubsystem;

    public ShootCommand(ShooterSubsystem subsystem){
        shooterSubsystem = subsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.flickReset();
    }

    @Override
    public void execute(){
        shooterSubsystem.flickReset();
        shooterSubsystem.flick();
    }

    public void stopShooter(){
        shooterSubsystem.stop();
    }

    public void returnHome(){
        shooterSubsystem.homePos();
    }
}
