package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
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
    private GamepadEx ringholderOp = new GamepadEx(gamepad2);
    private GamepadEx ringpusherOp = new GamepadEx(gamepad1);
    private GamepadEx shooterOp = new GamepadEx(gamepad2);

    private DriveSubsystem driver = new DriveSubsystem(hardwareMap, "drive");

    private IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, "intakeMotor");
    private GamepadButton intakeRingButton = new GamepadButton(intakeOp, GamepadKeys.Button.A);
    private GamepadButton releaseRingButton = new GamepadButton(intakeOp, GamepadKeys.Button.B); //g1

    private RingHolderSubsystem ringholder = new RingHolderSubsystem(hardwareMap, "ringHolderServo");
    private GamepadButton moveRingHolderButton = new GamepadButton(ringholderOp, GamepadKeys.Button.A); //g2

    private RingPusherSubsystem ringpusher = new RingPusherSubsystem(hardwareMap, "ringPusherServo");
    private GamepadButton pushRingButton = new GamepadButton(ringpusherOp, GamepadKeys.Button.X); //g1

    private ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, "shooterMotor");
    private GamepadButton shooterButton = new GamepadButton(shooterOp, GamepadKeys.Button.A); // g2

    @Override
    public void initialize() {
        /*intakeRingButton.whenPressed(new InstantCommand(intake::intake, intake));
        releaseRingButton.whenPressed(new InstantCommand(intake::release, intake));
        */

        intakeRingButton.whenPressed(intake::intakeRing);
        releaseRingButton.whenPressed(intake::releaseRing);

        moveRingHolderButton.whenPressed(ringholder::moveRingHolder);

        pushRingButton.whenPressed(ringpusher::pushRing);

        shooterButton.whenPressed(shooter::startShooter);

        register(intake, ringholder, ringpusher, shooter);
    }
}
