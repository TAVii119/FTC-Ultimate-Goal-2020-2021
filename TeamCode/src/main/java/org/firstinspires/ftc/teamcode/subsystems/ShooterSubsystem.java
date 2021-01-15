package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

/*
This class was created by Botosan Octavian on January 14, 2021.
This is a subsystem for the ring intake that we use.
 */

public class ShooterSubsystem extends SubsystemBase {

    private Motor shooterFrontMotor;
    private Motor shooterBackMotor;
    private Telemetry telemetry;
    private DoubleSupplier power;
    private boolean shooterActive;

    public ShooterSubsystem(Motor shooterFront, Motor shooterBack, Telemetry telemetryIn, DoubleSupplier getPower) {
        shooterFrontMotor = shooterFront;
        shooterBackMotor = shooterBack;
        telemetry = telemetryIn;
        power = getPower;
        shooterActive = true;

    }

    public void shoot() {
        shooterFrontMotor.set(power.getAsDouble());
        shooterBackMotor.set(power.getAsDouble());
        shooterActive = true;
    }

    public void stop() {
        shooterFrontMotor.stopMotor();
        shooterBackMotor.stopMotor();
        shooterActive = false;
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter power", power.getAsDouble());
        telemetry.addData("Shooter active", shooterActive);
        telemetry.update();
    }
}