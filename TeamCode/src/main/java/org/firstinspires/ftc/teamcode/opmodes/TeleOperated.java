package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.RingLiftCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.WobbleMotorCommand;
import org.firstinspires.ftc.teamcode.commands.WobbleServoCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RingLiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

/*
This class was created by Botosan Octavian on October 28, 2020.
This class is for the 2 minutes Tele-Operated period.
The robot is controlled by two robot operated using Playstation DualShock 4 controller.
 */

@TeleOp(name="TeleOp")
public class TeleOperated extends CommandOpMode {

    // Servos and Motors
    private Motor fL, fR, bL, bR;
    private Motor shooterFront, shooterBack, intake, wobbleMotor;
    private MotorGroup flywheel;
    private Servo loaderFront, loaderBack, wobbleServo;

    // Subsystems
    private DriveSubsystem driveSystem;
    private ShooterSubsystem shooterSystem;
    private IntakeSubsystem intakeSystem;
    private RingLiftSubsystem ringLiftSystem;
    private WobbleSubsystem wobbleSystem;

    // Commands
    private DriveCommand driveCommand;
    private ShootCommand shootCommand;
    private IntakeCommand intakeCommand;
    private OuttakeCommand outtakeCommand;
    private InstantCommand runFlyWheelCommand;
    private RingLiftCommand ringLiftCommand;
    private WobbleMotorCommand wobbleMotorCommand;
    private WobbleServoCommand wobbleServoCommand;

    // Extra
    private GamepadEx driverOp, driver2;
    private Button intakeButton, outtakeButton, shootCommandGroupButton, slowDriveButton, ringLiftButton, wobbleServoButton,
    shootButton, stopShootButton;
    private FtcDashboard dashboard;
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

        wobbleMotor = new Motor(hardwareMap, "wobbleMotor", Motor.GoBILDA.RPM_312);

        loaderFront = hardwareMap.get(Servo.class, "loaderFrontServo");
        loaderBack = hardwareMap.get(Servo.class, "loaderBackServo");
        loaderBack.setDirection(Servo.Direction.REVERSE);

        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");

        // Controller
        driverOp = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        dashboard = FtcDashboard.getInstance();

        //Subsystems and Commands
        driveSystem = new DriveSubsystem(fL, fR, bL, bR);
        driveCommand = new DriveCommand(driveSystem, driverOp::getLeftX, driverOp::getLeftY, driverOp::getRightX, () -> mult);

        shooterSystem = new ShooterSubsystem(flywheel, telemetry);
        shootCommand = new ShootCommand(shooterSystem);

        intakeSystem = new IntakeSubsystem(intake);
        intakeCommand = new IntakeCommand(intakeSystem);
        outtakeCommand = new OuttakeCommand(intakeSystem);

        ringLiftSystem = new RingLiftSubsystem(loaderFront, loaderBack);
        ringLiftCommand = new RingLiftCommand(ringLiftSystem);

        wobbleSystem = new WobbleSubsystem(wobbleServo, wobbleMotor, telemetry);
        wobbleMotorCommand = new WobbleMotorCommand(wobbleSystem, gamepad2.left_stick_y);
        wobbleServoCommand = new WobbleServoCommand(wobbleSystem);

        /*m_driverOp.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(()->mult = 0.5, ()->mult = 1.0);
        m_driverOp.getGamepadButton(GamepadKeys.Button.A).whenHeld(shootCommandGroup);

        m_driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(intakeCommand);
        m_driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(outtakeCommand);
         */

        intakeButton = new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER).whenHeld(intakeCommand);
        outtakeButton = new GamepadButton(driverOp, GamepadKeys.Button.LEFT_BUMPER).whenHeld(outtakeCommand);

        ringLiftButton = new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(ringLiftCommand);
        wobbleServoButton = new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(wobbleServoCommand);

        register(driveSystem, intakeSystem, shooterSystem, ringLiftSystem, wobbleSystem);
        driveSystem.setDefaultCommand(driveCommand);
    }
}
