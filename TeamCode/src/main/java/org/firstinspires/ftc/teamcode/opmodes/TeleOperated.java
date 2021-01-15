package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.BetterToggle;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.GamepadButtonB;
import org.firstinspires.ftc.teamcode.commands.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.commands.NoShootCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeStartCommand;
import org.firstinspires.ftc.teamcode.commands.PickUpCommand;
import org.firstinspires.ftc.teamcode.commands.PutDownCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

/*
This class was created by Botosan Octavian on October 28, 2020.
This class is for the 2 minutes Tele-Operated period.
The robot is controlled by two robot operated using Playstation DualShock 4 controller.
 */

@TeleOp(name="TeleOp")
public class TeleOperated extends CommandOpMode {

    public double pwrSelect = 1.0;

    private Motor fL, bL, fR, bR;
    private Motor shooterFront, shooterBack, intake, wobble;
    private Servo wobbleServo;

    private DriveSubsystem mecDrive;
    private DriveCommand driveCommand;

    private ShooterSubsystem shooterSystem;
    private ShootCommand shootCommand;
    private NoShootCommand stopCommand;

    private IntakeSubsystem intakeSystem;
    private IntakeStartCommand intakeStartCommand;
    private OuttakeStartCommand outtakeStartCommand;

    private WobbleSubsystem wobbleSystem;
    private PickUpCommand pickUpCommand;
    private PutDownCommand putDownCommand;

    private GamepadEx m_driverOp, m_toolOp;
    private Button toggleShooter, dpadUp, dpadDown, intakeOn, outtakeOn, wobbleButton, wobbleTwo;
    private Trigger leftTrigger, rightTrigger;
    private TriggerReader leftTriggerReader, rightTriggerReader;
    private BetterToggle toggleWobble, toggleTestTwo;
    private ElapsedTime elapsedTime;
    private FunctionalCommand openCommand, closeCommand;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        intake = new Motor(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_312);
        shooterFront = new Motor(hardwareMap, "frontShooterMotor", Motor.GoBILDA.BARE);
        shooterBack = new Motor(hardwareMap, "backShooterMotor", Motor.GoBILDA.BARE);
        wobble = new Motor(hardwareMap, "wobble");
        wobble.setRunMode(Motor.RunMode.PositionControl);
        wobble.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");

        mecDrive = new DriveSubsystem(fL, fR, bL, bR);

        m_driverOp = new GamepadEx(gamepad1);
        m_toolOp = new GamepadEx(gamepad2);
        elapsedTime = new ElapsedTime();


        dpadDown = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    if (pwrSelect < 0.05) {
                        pwrSelect = 1;
                    } else {
                        pwrSelect -= 0.05;
                    }
                }));
        dpadUp = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (pwrSelect > 0.95) {
                        pwrSelect = 0;
                    } else {
                        pwrSelect += 0.05;
                    }
                }));

        driveCommand = new DriveCommand(mecDrive, m_driverOp::getLeftX, m_driverOp::getLeftY, m_driverOp::getRightX);

        shooterSystem = new ShooterSubsystem(shooterFront, shooterBack, telemetry, () -> pwrSelect);
        shootCommand = new ShootCommand(shooterSystem);
        stopCommand = new NoShootCommand(shooterSystem);
        toggleShooter = new GamepadButton(m_driverOp, GamepadKeys.Button.A).toggleWhenPressed(shootCommand);

        intakeSystem = new IntakeSubsystem(intake);
        intakeStartCommand = new IntakeStartCommand(intakeSystem);
        outtakeStartCommand = new OuttakeStartCommand(intakeSystem);

        intakeOn = new GamepadButton(m_driverOp, GamepadKeys.Button.RIGHT_BUMPER).whenHeld(intakeStartCommand);
        outtakeOn = new GamepadButton(m_driverOp, GamepadKeys.Button.LEFT_BUMPER).whenHeld(outtakeStartCommand);

        wobbleSystem = new WobbleSubsystem(wobbleServo, wobble, telemetry);
        pickUpCommand = new PickUpCommand(wobbleSystem, elapsedTime);
        putDownCommand = new PutDownCommand(wobbleSystem, elapsedTime);
        toggleWobble = new GamepadButtonB(m_driverOp, GamepadKeys.Button.X).toggleWhenPressed(putDownCommand, pickUpCommand);

        leftTriggerReader = new TriggerReader(m_driverOp, GamepadKeys.Trigger.LEFT_TRIGGER);
        rightTriggerReader = new TriggerReader(m_driverOp, GamepadKeys.Trigger.RIGHT_TRIGGER);

        mecDrive.setDefaultCommand(driveCommand);
    }
}
