package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

public class SequentialShooter extends SequentialCommandGroup {
    private ShootCommand shooterCommand;

    public SequentialShooter(InstantCommand runFlyWheelCommand, WaitCommand waitCommand,
                             ShootCommand shooterCommand) {
        this.shooterCommand = shooterCommand;

        addCommands(runFlyWheelCommand, waitCommand, shooterCommand);
    }

    @Override
    public void end(boolean interrupted){
        shooterCommand.stopShooter();
        shooterCommand.returnHome();
    }
}
