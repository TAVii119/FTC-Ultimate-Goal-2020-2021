package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleSupplier;

/*
This class was created by Botosan Octavian on October 28, 2020.
This is a subsystem for the Mecanum drivetrain that we use.
Mecanum drivetrains are holonomic (they can move in any direction and also rotate in place).
 */

public class DriveSubsystem extends SubsystemBase {

    private MecanumDrive drive;
    private Motor fL, bL, fR, bR;

    public DriveSubsystem(Motor frontL, Motor frontR, Motor backL, Motor backR) {
        fL = frontL;
        fR = frontR;
        bL = backL;
        bR = backR;
        drive = new MecanumDrive(fL, fR, bL, bR);
    }

    //Forward Speed, Turn Speed and Strafe Speed,
    public void drive(double forwardSpeed, double turnSpeed, double strafeSpeed) {
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed, true);
    }
}
