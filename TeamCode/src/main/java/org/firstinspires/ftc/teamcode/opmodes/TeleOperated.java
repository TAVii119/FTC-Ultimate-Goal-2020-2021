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
import org.firstinspires.ftc.teamcode.commands.FlickerCommand;
import org.firstinspires.ftc.teamcode.commands.LiftRampCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LowerRampCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.WobblePickUpCommand;
import org.firstinspires.ftc.teamcode.commands.WobblePutDownCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RampSubsystem;
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
    private Motor intake, wobbleMotor;
    private MotorGroup flywheel;
    private Servo loaderFront, loaderBack, wobbleServo, flickerServo;

    // Subsystems
    private DriveSubsystem driveSystem;
    private ShooterSubsystem shooterSystem;
    private IntakeSubsystem intakeSystem;
    private RingLiftSubsystem ringLiftSystem;
    private FlickerSubsystem flickerSystem;
    private WobbleSubsystem wobbleSystem;
    private RampSubsystem rampSystem;

    // Commands
    private DriveCommand driveCommand;
    private ShootCommand shootCommand;
    private IntakeCommand intakeCommand;
    private OuttakeCommand outtakeCommand;
    private FlickerCommand flickerCommand;
    private InstantCommand ringLiftCommand;
    private WobblePickUpCommand wobblePickUpCommand;
    private WobblePutDownCommand wobblePutDownCommand;
    private InstantCommand grabberCommand;
    private LiftRampCommand liftRampCommand;
    private LowerRampCommand lowerRampCommand;

    // Extra
    private GamepadEx driver1, driver2;
    private Button intakeButton, outtakeButton, ringLiftButton, shootButton,
            flickButton, wobbleGrabberButton, wobblePickUpButton, wobblePutDownButton,
            liftRampButton, lowerRampButton;
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
        loaderBack.setDirection(Servo.Direction.REVERSE);

        // Controller
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        dashboard = FtcDashboard.getInstance();

        //Subsystems and Commands
        driveSystem = new DriveSubsystem(fL, fR, bL, bR);
        driveCommand = new DriveCommand(driveSystem, driver1::getLeftY, driver1::getRightX, driver1::getLeftX, mult);

        if (gamepad1.left_stick_button)
            mult = .3;
        else if (gamepad1.right_stick_button)
            mult = 1;

        shooterSystem = new ShooterSubsystem(flywheel, telemetry);
        shootCommand = new ShootCommand(shooterSystem);

        intakeSystem = new IntakeSubsystem(intake);
        intakeCommand = new IntakeCommand(intakeSystem);
        outtakeCommand = new OuttakeCommand(intakeSystem);

        ringLiftSystem = new RingLiftSubsystem(loaderFront, loaderBack);
        ringLiftCommand = new InstantCommand(()-> {
            if (ringLiftSystem.isLiftUp())
                ringLiftSystem.returnRingLift();
            else ringLiftSystem.moveRingLift();
        }, ringLiftSystem);

        flickerSystem = new FlickerSubsystem(hardwareMap, "feederServo");
        flickerCommand = new FlickerCommand(flickerSystem);

        wobbleSystem = new WobbleSubsystem(wobbleMotor, wobbleServo);
        grabberCommand = new InstantCommand(()-> {
            if (wobbleSystem.isGrabbing())
                wobbleSystem.openGrabber();
            else
                wobbleSystem.closeGrabber();
        }, wobbleSystem);
        wobblePickUpCommand = new WobblePickUpCommand(wobbleSystem);
        wobblePutDownCommand = new WobblePutDownCommand(wobbleSystem);

        intakeButton = new GamepadButton(driver1, GamepadKeys.Button.A).whenHeld(intakeCommand);
        outtakeButton = new GamepadButton(driver1, GamepadKeys.Button.B).whenHeld(outtakeCommand);

        ringLiftButton = new GamepadButton(driver2, GamepadKeys.Button.X).whenPressed(ringLiftCommand);
        flickButton = new GamepadButton(driver2, GamepadKeys.Button.A).whileHeld(flickerCommand);
        shootButton = new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(shootCommand);
        wobbleGrabberButton = new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(grabberCommand);
        wobblePickUpButton = new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(wobblePickUpCommand);
        wobblePutDownButton = new GamepadButton(driver2, GamepadKeys.Button.LEFT_BUMPER).whenPressed(wobblePutDownCommand);
        liftRampButton = new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP).whenPressed(liftRampCommand);
        lowerRampButton = new GamepadButton(driver2, GamepadKeys.Button.DPAD_DOWN).whenPressed(lowerRampCommand);

        register(driveSystem);
        driveSystem.setDefaultCommand(driveCommand);
    }
}
