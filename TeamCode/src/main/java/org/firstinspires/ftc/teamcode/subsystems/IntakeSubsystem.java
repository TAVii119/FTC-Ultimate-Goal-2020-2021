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

    private Motor intakeMotor;
    private Telemetry telemetry;

    public IntakeSubsystem(Motor IntakeMotor) {
        intakeMotor = IntakeMotor;
    }

    public void suck() {
        intakeMotor.set(-0.9);
    }
    public void down(){
        intakeMotor.set(0.9);
    }
    public void stop() {
        intakeMotor.stopMotor();
    }

}