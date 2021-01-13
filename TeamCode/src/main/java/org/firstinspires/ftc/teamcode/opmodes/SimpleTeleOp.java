package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * This class was created by Botosan Octavian on January, 2021.
 * This class is for the 2 minutes Tele-Operated period.
 * The robot is controlled by two robot operated using Playstation DualShock 4 controller.
 *
 * Does the same thing as {@link TeleOperated} but is much simpler
 * in scope.
 *
 * Note that the <b>proper</b> way to do this is with the SampleTeleOp version,
 * where most things are set up as commands/subsystems to avoid potential drawbacks
 * of the {@link InstantCommand}.
 */

@TeleOp
//@Disabled
public class SimpleTeleOp extends CommandOpMode {

    static final double WHEEL_DIAMETER = 96; // millimeters

    private GamepadEx driverOp = new GamepadEx(gamepad1);
    private GamepadEx intakeOp = new GamepadEx(gamepad1);
    private MotorEx flMotor, frMotor, blMotor, brMotor;
    private DriveSubsystem drive = new DriveSubsystem(flMotor, frMotor, blMotor, brMotor, WHEEL_DIAMETER);
    private Drivetrain driveCommand = new Drivetrain(drive, ()->driverOp.getLeftY(), ()->driverOp.getRightX(), ()->driverOp.getLeftX());
    private GamepadButton intakeRingButton = new GamepadButton(intakeOp, GamepadKeys.Button.A);
    private GamepadButton releaseRingButton = new GamepadButton(intakeOp, GamepadKeys.Button.B);
    private IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, "intake");

    @Override
    public void initialize() {
        register(drive, intake);

        // using InstantCommand here is not the greatest idea because the servos move in nonzero time
        // alternatives are adding WaitUntilCommands or making these commands.
        // As a result of this uncertainty, we add the gripper subsystem to ensure requirements are met.
        intakeRingButton.whenPressed(new InstantCommand(intake::intakeRing, intake));
        releaseRingButton.whenPressed(new InstantCommand(intake::releaseRing, intake));

        drive.setDefaultCommand(driveCommand);
    }
}