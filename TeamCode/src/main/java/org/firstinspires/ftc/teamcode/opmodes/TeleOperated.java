package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FlickerCommand;
import org.firstinspires.ftc.teamcode.commands.FlickerOnceCommand;
import org.firstinspires.ftc.teamcode.commands.LiftRampCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LowerRampCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.UpperRampCommand;
import org.firstinspires.ftc.teamcode.commands.WobbleCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RampSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RingBlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RingLiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
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
    private Motor intake, wobbleMotor;
    private MotorGroup flywheel;
    private Servo loaderFront, loaderBack, wobbleServo, wobbleServo2, flickerServo, shooterServo, ringBlockerLeft, ringBlockerRight;

    // Subsystems
    private DriveSubsystem driveSystem;
    private ShooterSubsystem shooterSystem;
    private IntakeSubsystem intakeSystem;
    private RingLiftSubsystem ringLiftSystem;
    private FlickerSubsystem flickerSystem;
    private WobbleSubsystem wobbleSystem;
    private RampSubsystem rampSystem;
    private RingBlockerSubsystem ringBlockerSystem;

    // Commands
    private DriveCommand driveCommand;
    private IntakeCommand intakeCommand;
    private OuttakeCommand outtakeCommand;
    private FlickerCommand flickerCommand;
    private FlickerOnceCommand flickerOnceCommand;
    private InstantCommand ringLiftCommand;
    private WobbleCommand wobbleCommand;
    private InstantCommand grabberCommand;
    private LiftRampCommand liftRampCommand;
    private LowerRampCommand lowerRampCommand;
    private UpperRampCommand upperRampCommand;
    private InstantCommand shootCommand;
    private InstantCommand slowShootCommand;
    private InstantCommand normalModeCommand;
    private InstantCommand towerAlignCommand;
    private InstantCommand rightPsAlignCommand;
    private InstantCommand centerPsAlignCommand;
    private InstantCommand leftPsAlignCommand;
    private InstantCommand resetRightPoseCommand;
    private InstantCommand resetLeftPoseCommand;
    private InstantCommand setRampPositionCommand;
    private InstantCommand ringBlockerCommand;
    private TimedAction flickerAction;

    // Extra
    private GamepadEx driver1, driver2;
    private Button intakeButton, outtakeButton, resetFlickButton, fastFlickButton, flickButton, ringLiftButton,
            shootButton, slowShootButton, wobbleGrabberButton, liftRampButton, lowerRampButton, normalModeButton, towerAlignButton,
            rightPsAlignButton, centerPsAlignButton, leftPsAlignButton, resetRightPoseButton, resetLeftPoseButton,
            ringBlockerButton, singleFlickButton, upperRampButton, flickOnceButton;
    private Trigger towerAlignTrigger;
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
        wobbleServo2 = hardwareMap.get(Servo.class, "wobbleServo2");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        flickerServo = hardwareMap.get(Servo.class, "feederServo");
        ringBlockerLeft = hardwareMap.get(Servo.class, "ringBlockerLeft");
        ringBlockerRight = hardwareMap.get(Servo.class, "ringBlockerRight");
        loaderBack.setDirection(Servo.Direction.REVERSE);
        shooterServo.setDirection(Servo.Direction.REVERSE);
        wobbleServo2.setDirection(Servo.Direction.REVERSE);
        ringBlockerRight.setDirection(Servo.Direction.REVERSE);

        // Controller
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        dashboard = FtcDashboard.getInstance();

        //Subsystems and Commands
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        driveSystem = new DriveSubsystem(drive, false, telemetry);
        driveCommand = new DriveCommand(driveSystem, driver1::getLeftY, driver1::getLeftX, driver1::getRightX);
        flickerOnceCommand = new FlickerOnceCommand(flickerSystem);

        normalModeCommand = new InstantCommand(()-> {
            driveSystem.normalMode();
        }, driveSystem);

        towerAlignCommand = new InstantCommand(()-> {
            driveSystem.alignToTower();
        }, driveSystem);

        rightPsAlignCommand = new InstantCommand(()-> {
            driveSystem.alignToRightPs();
        }, driveSystem);

        centerPsAlignCommand = new InstantCommand(()-> {
            driveSystem.alignToCenterPs();
        }, driveSystem);

        leftPsAlignCommand = new InstantCommand(()-> {
            driveSystem.alignToLeftPs();
        }, driveSystem);

        resetRightPoseCommand = new InstantCommand(()-> {
            driveSystem.setPoseEstimate(new Pose2d(-9, -63.0));
        }, driveSystem);

        resetLeftPoseCommand = new InstantCommand(()-> {
            driveSystem.setPoseEstimate(new Pose2d(-9, 14.0));
        }, driveSystem);

        setRampPositionCommand = new InstantCommand(()-> {
            rampSystem.rampPos(driveSystem.setRampPosition());
        }, rampSystem, driveSystem);

        shooterSystem = new ShooterSubsystem(flywheel);
        shootCommand = new InstantCommand(()-> {
            if (!shooterSystem.isShooting()) {
                shooterSystem.shoot();
                ringLiftSystem.moveRingLift();
                rampSystem.topGoalPos();
                ringBlockerSystem.unBlockRings();
            } else {
                shooterSystem.stopShoot();
                ringLiftSystem.returnRingLift();
                ringBlockerSystem.blockRings();
            }
        }, shooterSystem, ringLiftSystem, rampSystem, ringBlockerSystem);

        slowShootCommand = new InstantCommand(()-> {
            shooterSystem.slowShoot();
            ringLiftSystem.moveRingLift();
            rampSystem.powershotPos();
            ringBlockerSystem.unBlockRings();
        }, shooterSystem, ringLiftSystem, rampSystem, ringBlockerSystem);

        intakeSystem = new IntakeSubsystem(intake);
        intakeCommand = new IntakeCommand(intakeSystem);
        outtakeCommand = new OuttakeCommand(intakeSystem);

        ringLiftSystem = new RingLiftSubsystem(loaderFront, loaderBack);
        ringLiftCommand = new InstantCommand(()-> {
            if (ringLiftSystem.isLiftUp())
                ringLiftSystem.returnRingLift();
            else ringLiftSystem.moveRingLift();
        }, ringLiftSystem);

        flickerAction = new TimedAction(
                ()->flickerServo.setPosition(0.32),
                ()->flickerServo.setPosition(0),
                200,
                true
        );

        flickerSystem = new FlickerSubsystem(flickerServo, flickerAction);
        flickerCommand = new FlickerCommand(flickerSystem);

        wobbleSystem = new WobbleSubsystem(wobbleMotor, wobbleServo, wobbleServo2);
        wobbleCommand = new WobbleCommand(wobbleSystem, driver2::getRightY);
        grabberCommand = new InstantCommand(()-> {
            if (wobbleSystem.isGrabbing())
                wobbleSystem.openGrabber();
            else
                wobbleSystem.closeGrabber();
        }, wobbleSystem);

        rampSystem = new RampSubsystem(shooterServo, telemetry);
        liftRampCommand = new LiftRampCommand(rampSystem);
        lowerRampCommand = new LowerRampCommand(rampSystem);
        upperRampCommand = new UpperRampCommand(rampSystem);

        ringBlockerSystem = new RingBlockerSubsystem(ringBlockerLeft, ringBlockerRight);
        ringBlockerCommand = new InstantCommand(()-> {
            if (ringBlockerSystem.isBlockerDown())
                ringBlockerSystem.initBlockRings();
            else ringBlockerSystem.blockRings();
        }, ringBlockerSystem);

        intakeButton = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenHeld(intakeCommand);
        outtakeButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenHeld(outtakeCommand);
        flickButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenHeld(flickerCommand);
        ringBlockerButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenPressed(ringBlockerCommand);
        normalModeButton = new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(normalModeCommand);
        leftPsAlignButton = new GamepadButton(driver1, GamepadKeys.Button.DPAD_LEFT).whenPressed(leftPsAlignCommand);
        centerPsAlignButton = new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP).whenPressed(centerPsAlignCommand);
        rightPsAlignButton = new GamepadButton(driver1, GamepadKeys.Button.DPAD_RIGHT).whenPressed(rightPsAlignCommand);
        resetLeftPoseButton = new GamepadButton(driver1, GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(resetLeftPoseCommand);
        resetRightPoseButton = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(resetRightPoseCommand);
        
        shootButton = new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(shootCommand);
        slowShootButton = new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(slowShootCommand);
        flickButton = new GamepadButton(driver2, GamepadKeys.Button.X).whenHeld(flickerCommand);
        ringBlockerButton = new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(ringBlockerCommand);
        wobbleGrabberButton = new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(grabberCommand);
        liftRampButton = new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP).whenPressed(liftRampCommand);
        lowerRampButton = new GamepadButton(driver2, GamepadKeys.Button.DPAD_DOWN).whenPressed(lowerRampCommand);
        upperRampButton = new GamepadButton(driver2, GamepadKeys.Button.DPAD_RIGHT).whenPressed(upperRampCommand);

        register(driveSystem, wobbleSystem, flickerSystem, intakeSystem, rampSystem, ringLiftSystem, shooterSystem, ringBlockerSystem);
        driveSystem.setDefaultCommand(driveCommand);
        wobbleSystem.setDefaultCommand(wobbleCommand);
    }
}