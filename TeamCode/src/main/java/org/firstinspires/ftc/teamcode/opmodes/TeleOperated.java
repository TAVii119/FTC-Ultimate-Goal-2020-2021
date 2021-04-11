package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.commands.AutoRampCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FlickerCommand;
import org.firstinspires.ftc.teamcode.commands.LiftRampCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LocalizationCommand;
import org.firstinspires.ftc.teamcode.commands.LowerRampCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.UpperRampCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RampSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RingBlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.TimedAction;

/*
This class was created by Botosan Octavian on October 28, 2020.
This class is for the 2 minutes Tele-Operated period.
The robot is controlled by two robot operated using Playstation DualShock 4 controller.
*/

@TeleOp(name="TeleOperated")
public class TeleOperated extends CommandOpMode {

    // Servos and Motors
    private Motor fL, fR, bL, bR;
    private Motor intake;
    private MotorGroup flywheel;
    private Servo flickerServo, shooterServo, turretServo, intakeServo, ringBlockerServoLeft, ringBlockerServoRight;

    // Subsystems
    private TurretSubsystem turretSystem;
    private DriveSubsystem driveSystem;
    private ShooterSubsystem shooterSystem;
    private IntakeSubsystem intakeSystem;
    private FlickerSubsystem flickerSystem;
    private RampSubsystem rampSystem;
    private RingBlockerSubsystem ringBlockerSystem;
    private LocalizationSubsystem localizationSystem;

    // Commands
    private AutoRampCommand autoRampCommand;
    private LocalizationCommand localizationCommand;
    private TurretCommand turretCommand;
    private DriveCommand driveCommand;
    private IntakeCommand intakeCommand;
    private OuttakeCommand outtakeCommand;
    private FlickerCommand flickerCommand;
    private InstantCommand grabberCommand;
    private LiftRampCommand liftRampCommand;
    private LowerRampCommand lowerRampCommand;
    private UpperRampCommand upperRampCommand;
    private InstantCommand shootCommand;
    private InstantCommand slowShootCommand;
    private InstantCommand setRampPositionCommand;
    private InstantCommand resetAlignmentCommand;
    private InstantCommand turretToPowershots;
    private InstantCommand liftIntakeCommand;
    private InstantCommand ringBlockerCommand;

    private InstantCommand towergoalAlignCommand;
    private InstantCommand leftPsAlignCommand;
    private InstantCommand centerPsAlignCommand;
    private InstantCommand rightPsAlignCommand;

    private TimedAction flickerAction;

    // Extra
    private GamepadEx driver1, driver2;
    private Button intakeButton, outtakeButton, resetFlickButton, fastFlickButton, flickButton, ringLiftButton,
            shootButton, slowShootButton, liftRampButton, lowerRampButton, normalModeButton, towerAlignButton,
            rightPsAlignButton, centerPsAlignButton, leftPsAlignButton, resetRightPoseButton, resetLeftPoseButton,
            singleFlickButton, upperRampButton, flickOnceButton, resetAlignmentButton, liftIntakeButton, powershotTurretButton,
            ringBlockersButton;
    private Trigger towerAlignTrigger;
    private static T265Camera slamra = null;

    @Override
    public void initialize() {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }

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

        // SERVOS
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        flickerServo = hardwareMap.get(Servo.class, "feederServo");
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        ringBlockerServoLeft = hardwareMap.get(Servo.class, "ringBlockerLeft");
        ringBlockerServoRight = hardwareMap.get(Servo.class, "ringBlockerRight");

        turretServo.setPosition(0.3);
        ringBlockerServoLeft.setPosition(0.1);
        ringBlockerServoRight.setPosition(0.1);


        flickerServo.setDirection(Servo.Direction.REVERSE);
        ringBlockerServoRight.setDirection(Servo.Direction.REVERSE);
        shooterServo.setDirection(Servo.Direction.REVERSE);

        // Controller
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        //Subsystems and Commands

        ringBlockerSystem = new RingBlockerSubsystem(ringBlockerServoLeft, ringBlockerServoRight);

        ringBlockerSystem.init();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        driveSystem = new DriveSubsystem(drive, false, telemetry);
        driveCommand = new DriveCommand(driveSystem, driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        setRampPositionCommand = new InstantCommand(()-> {
            rampSystem.rampPos(driveSystem.setRampPosition());
        }, rampSystem, driveSystem);

        shooterSystem = new ShooterSubsystem(flywheel);
        shootCommand = new InstantCommand(()-> {
            if (!shooterSystem.isShooting()) {
                rampSystem.topGoalPos();
                shooterSystem.shoot();
            } else {
                shooterSystem.stopShoot();
            }
        }, shooterSystem, rampSystem);

        slowShootCommand = new InstantCommand(()-> {
            shooterSystem.slowShoot();
            rampSystem.powershotPos();
        }, shooterSystem, rampSystem);

        intakeSystem = new IntakeSubsystem(intake, intakeServo);
        intakeCommand = new IntakeCommand(intakeSystem);
        outtakeCommand = new OuttakeCommand(intakeSystem);

        flickerAction = new TimedAction(
                ()->flickerServo.setPosition(0.165),
                ()->flickerServo.setPosition(0),
                200,
                true
        );

        flickerSystem = new FlickerSubsystem(flickerServo, flickerAction);
        flickerCommand = new FlickerCommand(flickerSystem);

        rampSystem = new RampSubsystem(shooterServo, telemetry);
        liftRampCommand = new LiftRampCommand(rampSystem);
        lowerRampCommand = new LowerRampCommand(rampSystem);
        upperRampCommand = new UpperRampCommand(rampSystem);

        localizationSystem = new LocalizationSubsystem(slamra, telemetry);
        localizationCommand = new LocalizationCommand(localizationSystem);
        turretSystem = new TurretSubsystem(turretServo, telemetry);
        turretCommand = new TurretCommand(turretSystem, localizationSystem);
        autoRampCommand = new AutoRampCommand(rampSystem, localizationSystem);

        resetAlignmentCommand = new InstantCommand(()-> {
            sleep(50);
        }, driveSystem);

        ringBlockerCommand = new InstantCommand(()-> {
            if (ringBlockerSystem.isBlockerDown()) {
                ringBlockerSystem.unBlockRings();
            } else {
                ringBlockerSystem.blockRings();
            }
        }, ringBlockerSystem);

        liftIntakeCommand = new InstantCommand(()-> {
            if (intakeSystem.getServoPosition() == 0.0) {
                intakeSystem.unLift();
            } else {
                intakeSystem.lift();
            }
            sleep(50);
        }, intakeSystem);

        towergoalAlignCommand = new InstantCommand(()-> {
            localizationSystem.setCurrentTarget(0);
        }, localizationSystem);

        leftPsAlignCommand = new InstantCommand(()-> {
            localizationSystem.setCurrentTarget(1);
        }, localizationSystem);

        centerPsAlignCommand = new InstantCommand(()-> {
            localizationSystem.setCurrentTarget(2);
        }, localizationSystem);

        rightPsAlignCommand = new InstantCommand(()-> {
            localizationSystem.setCurrentTarget(3);
        }, localizationSystem);

        intakeButton = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenHeld(intakeCommand);
        outtakeButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenHeld(outtakeCommand);
        resetAlignmentButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(resetAlignmentCommand);
        liftIntakeButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenPressed(liftIntakeCommand);
        ringBlockersButton = new GamepadButton(driver1, GamepadKeys.Button.X).whenPressed(ringBlockerCommand);
        towerAlignButton = new GamepadButton(driver1, GamepadKeys.Button.DPAD_DOWN).whenPressed(towergoalAlignCommand);
        leftPsAlignButton = new GamepadButton(driver1, GamepadKeys.Button.DPAD_LEFT).whenPressed(leftPsAlignCommand);
        centerPsAlignButton = new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP).whenPressed(centerPsAlignCommand);
        rightPsAlignButton = new GamepadButton(driver1, GamepadKeys.Button.DPAD_RIGHT).whenPressed(rightPsAlignCommand);

        shootButton = new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(shootCommand);
        slowShootButton = new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(slowShootCommand);
        flickButton = new GamepadButton(driver2, GamepadKeys.Button.X).whenHeld(flickerCommand);
        liftRampButton = new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP).whenPressed(liftRampCommand);
        lowerRampButton = new GamepadButton(driver2, GamepadKeys.Button.DPAD_DOWN).whenPressed(lowerRampCommand);
        upperRampButton = new GamepadButton(driver2, GamepadKeys.Button.DPAD_RIGHT).whenPressed(upperRampCommand);

        register(driveSystem, flickerSystem, intakeSystem, rampSystem, shooterSystem, turretSystem, localizationSystem, ringBlockerSystem);
        driveSystem.setDefaultCommand(driveCommand);
        localizationSystem.setDefaultCommand(localizationCommand);
        turretSystem.setDefaultCommand(turretCommand);
        rampSystem.setDefaultCommand(autoRampCommand);
    }
}