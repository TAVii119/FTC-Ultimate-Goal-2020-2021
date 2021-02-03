package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.TimedAction;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

import static org.firstinspires.ftc.teamcode.drive.opmode.SampleLinkedPIDUse.kA;
import static org.firstinspires.ftc.teamcode.drive.opmode.SampleLinkedPIDUse.kStatic;

/*
This class was created by Botosan Octavian on January 14, 2021.
This is a subsystem for the shooter we use.
 */

public class ShooterSubsystem extends SubsystemBase {

    private MotorGroup flywheel;
    private double power = 0;
    private Telemetry telemetry;
    boolean isShooting = false;

    public ShooterSubsystem(MotorGroup flywheel, Telemetry telemetry){
        this.flywheel = flywheel;

        this.flywheel.setRunMode(Motor.RunMode.VelocityControl);
        this.flywheel.setVeloCoefficients(0.003, 0, 0.0001);
        this.flywheel.setFeedforwardCoefficients(0, 0.00042);
        this.telemetry = telemetry;
    }

    public void shoot() {
        flywheel.set(1);
        isShooting = true;
    }

    public void slowShoot() {
        flywheel.set(0.7);
        isShooting = true;
    }

    public void stopShoot() {
        flywheel.set(0);
        isShooting = false;
    }

    public boolean isShooting() {
        return isShooting;
    }
}