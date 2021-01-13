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
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;

/**
 * Does the same thing as {@link SimpleTeleOp} but is much simpler
 * in scope.
 *
 * Note that the <b>proper</b> way to do this is with the SampleTeleOp version,
 * where most things are set up as commands/subsystems to avoid potential drawbacks
 * of the {@link InstantCommand}.
 */
@TeleOp
@Disabled
public class SimpleTeleOp extends CommandOpMode {

    static final double WHEEL_DIAMETER = 96; // millimeters

    private GamepadEx toolOp = new GamepadEx(gamepad2);
    private GamepadButton grabButton = new GamepadButton(toolOp, GamepadKeys.Button.A);
    private GamepadButton releaseButton = new GamepadButton(toolOp, GamepadKeys.Button.B);
    private GripperSubsystem gripper = new GripperSubsystem(hardwareMap, "gripper");
    private MotorEx flMotor, frMotor, blMotor, brMotor;
    private DriveSubsystem drive;
    private GamepadEx driverOp = new GamepadEx(gamepad1);
    private Drivetrain driveCommand;
    /*private DriveSubsystem drive =
            new DriveSubsystem(hardwareMap, "left", "right", 100.0);
    private DefaultDrive driveCommand = new DefaultDrive(drive, driverOp::getLeftY, driverOp::getRightX);
    */


    @Override
    public void initialize() {
        flMotor = new MotorEx(hardwareMap, "flMotor");
        frMotor = new MotorEx(hardwareMap, "frMotor");
        blMotor = new MotorEx(hardwareMap, "blMotor");
        brMotor = new MotorEx(hardwareMap, "brMotor");

        drive = new DriveSubsystem(flMotor, frMotor, blMotor, brMotor, WHEEL_DIAMETER);
        driveCommand = new Drivetrain(drive, ()->driverOp.getLeftY(), ()->driverOp.getRightX(), ()->driverOp.getLeftX());
        drive.setDefaultCommand(driveCommand);

        register(drive, gripper);
        
        // using InstantCommand here is not the greatest idea because the servos move in nonzero time
        // alternatives are adding WaitUntilCommands or making these commands.
        // As a result of this uncertainty, we add the gripper subsystem to ensure requirements are met.
        grabButton.whenPressed(new InstantCommand(gripper::grab, gripper));
        releaseButton.whenPressed(new InstantCommand(gripper::release, gripper));

        drive.setDefaultCommand(driveCommand);
    }

}