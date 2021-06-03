package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
This class was created by Botosan Octavian on January 14, 2021.
This is a subsystem for the ring intake that we use.
 */

public class IntakeSubsystem extends SubsystemBase {

    private Motor intakeMotor, intakeMotor2;
    private Telemetry telemetry;

    public IntakeSubsystem(Motor IntakeMotor, Motor IntakeMotor2) {
        intakeMotor = IntakeMotor;
        intakeMotor2 = IntakeMotor2;
    }

    public void intake() {
        intakeMotor.set(1);
        intakeMotor2.set(1);
    }
    public void outtake() {
        intakeMotor.set(-1);
        intakeMotor2.set(-1);
    }

    public void stop() {
        intakeMotor.stopMotor();
        intakeMotor2.stopMotor();
    }

}