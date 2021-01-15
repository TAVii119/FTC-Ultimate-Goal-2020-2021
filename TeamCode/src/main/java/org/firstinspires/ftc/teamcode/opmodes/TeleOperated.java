package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.RingLiftCommand;
import org.firstinspires.ftc.teamcode.commands.SequentialShooter;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RingLiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.TimedAction;

/*
This class was created by Botosan Octavian on October 28, 2020.
This class is for the 2 minutes Tele-Operated period.
The robot is controlled by two robot operated using Playstation DualShock 4 controller.
 */

@TeleOp(name="TeleOp")
public class TeleOperated extends CommandOpMode {

    // Servos and Motors
    private Motor fL, fR, bL, bR;
    private Motor shooterFront, shooterBack, intake;
    private MotorGroup flywheel;
    private SimpleServo flicker;
    private Servo loaderFront, loaderBack;

    // Subsystems
    private DriveSubsystem driveSystem;
    private ShooterSubsystem shooterSystem;
    private IntakeSubsystem intakeSystem;
    private RingLiftSubsystem ringLiftSystem;

    // Commands
    private DriveCommand driveCommand;
    private ShootCommand shooterCommand;
    private IntakeCommand intakeCommand;
    private OuttakeCommand outtakeCommand;
    private SequentialShooter shootCommandGroup;
    private InstantCommand runFlyWheelCommand;
    private RingLiftCommand ringLiftCommand;

    // Extra
    private GamepadEx driverOp, driver2;
    private Button intakeButton, outtakeButton, shootCommandGroupButton, slowDriveButton, ringLiftButton;
    private FtcDashboard dashboard;
    private TimedAction flickerAction;
    public double mult = 1.0;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "flMotor", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "frMotor", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "blMotor", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "brMotor", Motor.GoBILDA.RPM_312);

        intake = new Motor(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_312);
        flywheel = new MotorGroup(
                new Motor(hardwareMap, "shooterFrontMotor", Motor.GoBILDA.BARE),
                new Motor(hardwareMap, "shooterBackMotor", Motor.GoBILDA.BARE)
        );

        loaderFront = hardwareMap.get(Servo.class, "loaderFrontServo");
        loaderBack = hardwareMap.get(Servo.class, "loaderBackServo");
        loaderBack.setDirection(Servo.Direction.REVERSE);

        // Controller
        driverOp = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        dashboard = FtcDashboard.getInstance();

        //FlickerAction
        flickerAction = new TimedAction(
                ()-> flicker.setPosition(0.5),
                ()-> flicker.setPosition(0.27),
                600,
                true
        );

        //Subsystems and Commands
        driveSystem = new DriveSubsystem(fL, fR, bL, bR);
        driveCommand = new DriveCommand(driveSystem, driverOp::getLeftX, driverOp::getLeftY, driverOp::getRightX, () -> mult);

        shooterSystem = new ShooterSubsystem(flywheel, flicker, flickerAction, telemetry);
        shooterCommand = new ShootCommand(shooterSystem);
        runFlyWheelCommand = new InstantCommand(shooterSystem::shoot, shooterSystem);
        shootCommandGroup = new SequentialShooter(runFlyWheelCommand, new WaitCommand(1500), shooterCommand);

        intakeSystem = new IntakeSubsystem(intake);
        intakeCommand = new IntakeCommand(intakeSystem);
        outtakeCommand = new OuttakeCommand(intakeSystem);

        ringLiftSystem = new RingLiftSubsystem(loaderFront, loaderBack);
        ringLiftCommand = new RingLiftCommand(ringLiftSystem);

        /*m_driverOp.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(()->mult = 0.5, ()->mult = 1.0);
        m_driverOp.getGamepadButton(GamepadKeys.Button.A).whenHeld(shootCommandGroup);

        m_driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(intakeCommand);
        m_driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(outtakeCommand);
         */

        shootCommandGroupButton = new GamepadButton(driverOp, GamepadKeys.Button.A).whenHeld(shootCommandGroup);
        intakeButton = new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER).whenHeld(intakeCommand);
        outtakeButton = new GamepadButton(driverOp, GamepadKeys.Button.LEFT_BUMPER).whenHeld(outtakeCommand);
        ringLiftButton = new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(ringLiftCommand);

        register(driveSystem, intakeSystem, shooterSystem, ringLiftSystem);
        driveSystem.setDefaultCommand(driveCommand);
    }
}
