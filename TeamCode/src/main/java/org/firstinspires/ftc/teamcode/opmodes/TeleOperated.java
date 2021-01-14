package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RingHolderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RingPusherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/*
This class was created by Botosan Octavian on October 28, 2020.
This class is for the 2 minutes Tele-Operated period.
The robot is controlled by two robot operated using Playstation DualShock 4 controller.
 */

@TeleOp(name="TeleOp")
public class TeleOperated extends CommandOpMode {

    static final double WHEEL_DIAMETER = 96; // millimeters

    private GamepadEx driverOp = new GamepadEx(gamepad1);
    private GamepadEx intakeOp = new GamepadEx(gamepad1);
    private GamepadEx ringholderOp = new GamepadEx(gamepad1);
    private GamepadEx ringpusherOp = new GamepadEx(gamepad1);
    private GamepadEx shooterOp = new GamepadEx(gamepad2);

    private MotorEx flMotor, frMotor, blMotor, brMotor;
    private DriveSubsystem drive;
    private Drivetrain driveCommand;

    private IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, "intakeMotor");
    private GamepadButton intakeRingButton = new GamepadButton(intakeOp, GamepadKeys.Button.A);
    private GamepadButton releaseRingButton = new GamepadButton(intakeOp, GamepadKeys.Button.B);

    private RingHolderSubsystem ringholder = new RingHolderSubsystem(hardwareMap, "ringHolderServo");
    private GamepadButton moveRingHolderButton = new GamepadButton(ringholderOp, GamepadKeys.Button.X);

    private RingPusherSubsystem ringpusher = new RingPusherSubsystem(hardwareMap, "ringPusherServo");
    private GamepadButton pushRingButton = new GamepadButton(ringpusherOp, GamepadKeys.Button.Y);

    private ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, "shooterMotor");
    private GamepadButton shooterButton = new GamepadButton(shooterOp, GamepadKeys.Button.A);

    @Override
    public void initialize() {
        flMotor = new MotorEx(hardwareMap, "flMotor");
        frMotor = new MotorEx(hardwareMap, "frMotor");
        blMotor = new MotorEx(hardwareMap, "blMotor");
        brMotor = new MotorEx(hardwareMap, "brMotor");

        drive = new DriveSubsystem(flMotor, frMotor, blMotor, brMotor, WHEEL_DIAMETER);
        driveCommand = new Drivetrain(drive, ()->driverOp.getLeftY(), ()->driverOp.getRightX(), ()->driverOp.getLeftX());

        /*intakeRingButton.whenPressed(new InstantCommand(intake::intake, intake));
        releaseRingButton.whenPressed(new InstantCommand(intake::release, intake));
        */

        intakeRingButton.whenPressed(intake::intakeRing);
        releaseRingButton.whenPressed(intake::releaseRing);

        moveRingHolderButton.whenPressed(ringholder::moveRingHolder);

        pushRingButton.whenPressed(ringpusher::pushRing);

        shooterButton.whenPressed(shooter::startShooter);

        register(drive, intake, ringholder, ringpusher, shooter);
        drive.setDefaultCommand(driveCommand);
    }
}
