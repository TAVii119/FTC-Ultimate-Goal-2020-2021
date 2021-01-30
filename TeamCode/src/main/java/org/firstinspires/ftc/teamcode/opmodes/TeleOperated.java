package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FlickReturnCommand;
import org.firstinspires.ftc.teamcode.commands.FlickerCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.RingLiftCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.WobbleMotorCommand;
import org.firstinspires.ftc.teamcode.commands.WobbleServoCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RingLiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
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
    private Motor intake, wobbleMotor;
    private MotorGroup flywheel;
    private Servo loaderFront, loaderBack, wobbleServo, flickerServo;

    // Subsystems
    private DriveSubsystem driveSystem;
    private ShooterSubsystem shooterSystem;
    private IntakeSubsystem intakeSystem;
    private RingLiftSubsystem ringLiftSystem;
    private WobbleSubsystem wobbleSystem;
    private FlickerSubsystem flickerSystem;

    // Commands
    private DriveCommand driveCommand;
    private ShootCommand shootCommand;
    private IntakeCommand intakeCommand;
    private OuttakeCommand outtakeCommand;
    private RingLiftCommand ringLiftCommand;
    private WobbleMotorCommand wobbleMotorCommand;
    private WobbleServoCommand wobbleServoCommand;
    private FlickerCommand flickerCommand;
    private FlickReturnCommand flickReturnCommand;

    // Extra
    private GamepadEx driver1, driver2;
    private Button intakeButton, outtakeButton, ringLiftButton, wobbleServoButton,
    shootButton, flickButton, slowDrive;
    private FtcDashboard dashboard;
    public double mult = 1.0;

    @Override
    public void initialize() {
        // MOTORS
        fL = new Motor(hardwareMap, "flMotor", Motor.GoBILDA.RPM_312);
        fL.setInverted(true);
        fR = new Motor(hardwareMap, "frMotor", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "blMotor", Motor.GoBILDA.RPM_312);
        bL.setInverted(true);
        bR = new Motor(hardwareMap, "brMotor", Motor.GoBILDA.RPM_312);
        intake = new Motor(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_312);
        intake.setInverted(true);
        flywheel = new MotorGroup(
                new Motor(hardwareMap, "shooterFrontMotor", Motor.GoBILDA.BARE),
                new Motor(hardwareMap, "shooterBackMotor", Motor.GoBILDA.BARE)
        );
        flywheel.setInverted(true);
        wobbleMotor = new Motor(hardwareMap, "wobbleMotor", Motor.GoBILDA.RPM_312);

        // SERVOS
        loaderFront = hardwareMap.get(Servo.class, "loaderFrontServo");
        loaderBack = hardwareMap.get(Servo.class, "loaderBackServo");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        //flickerServo = hardwareMap.get(Servo.class, "feederServo");
        wobbleServo.setDirection(Servo.Direction.FORWARD);
        loaderFront.setDirection(Servo.Direction.FORWARD);
        loaderBack.setDirection(Servo.Direction.REVERSE);
        //flickerServo.setDirection(Servo.Direction.FORWARD);

        // Controller
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        dashboard = FtcDashboard.getInstance();

        //Subsystems and Commands
        driveSystem = new DriveSubsystem(fL, fR, bL, bR);
        driveCommand = new DriveCommand(driveSystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX);

        shooterSystem = new ShooterSubsystem(flywheel, telemetry);
        shootCommand = new ShootCommand(shooterSystem);

        intakeSystem = new IntakeSubsystem(intake);
        intakeCommand = new IntakeCommand(intakeSystem);
        outtakeCommand = new OuttakeCommand(intakeSystem);

        ringLiftSystem = new RingLiftSubsystem(loaderFront, loaderBack);
        ringLiftCommand = new RingLiftCommand(ringLiftSystem);

        wobbleSystem = new WobbleSubsystem(wobbleServo, wobbleMotor, telemetry);
        wobbleMotorCommand = new WobbleMotorCommand(wobbleSystem, -gamepad2.left_stick_y);
        wobbleServoCommand = new WobbleServoCommand(wobbleSystem);

        flickerSystem = new FlickerSubsystem(hardwareMap, "feederServo");
        flickerCommand = new FlickerCommand(flickerSystem);
        flickReturnCommand = new FlickReturnCommand(flickerSystem);

        intakeButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenHeld(intakeCommand);
        outtakeButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenHeld(outtakeCommand);

        ringLiftButton = new GamepadButton(driver2, GamepadKeys.Button.X).whenPressed(ringLiftCommand);
        wobbleServoButton = new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(wobbleServoCommand);
        flickButton = new GamepadButton(driver2, GamepadKeys.Button.A).whileHeld(flickerCommand);
        shootButton = new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(shootCommand);

        register(driveSystem);
        driveSystem.setDefaultCommand(driveCommand);
    }
}
