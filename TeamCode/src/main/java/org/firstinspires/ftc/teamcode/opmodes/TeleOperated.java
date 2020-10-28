package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/*
This class was created by Botosan Octavian on October 28, 2020.
This class is for the 2 minutes Tele-Operated period.
The robot is controlled by two robot operated using Playstation DualShock 4 controller.
 */
@TeleOp(name="TeleOp")
public class TeleOperated extends CommandOpMode {

    static final double WHEEL_DIAMETER = 96; // millimeters

    private MotorEx m_flMotor, m_frMotor, m_blMotor, m_brMotor;
    private DriveSubsystem m_drive;
    private GamepadEx m_driverOp;
    private MecanumDrive m_driveCommand;

    @Override
    public void initialize() {
        m_flMotor = new MotorEx(hardwareMap, "flMotor");
        m_frMotor = new MotorEx(hardwareMap, "frMotor");
        m_blMotor = new MotorEx(hardwareMap, "blMotor");
        m_brMotor = new MotorEx(hardwareMap, "brMotor");

        m_drive = new DriveSubsystem(m_flMotor, m_frMotor, m_blMotor, m_brMotor, WHEEL_DIAMETER);
        m_driverOp = new GamepadEx(gamepad1);
        m_driveCommand = new MecanumDrive(m_drive, ()->m_driverOp.getLeftY(), ()->m_driverOp.getLeftX(), ()->m_driverOp.getRightX());
        m_drive.setDefaultCommand(m_driveCommand);
        register(m_drive);
    }
}
