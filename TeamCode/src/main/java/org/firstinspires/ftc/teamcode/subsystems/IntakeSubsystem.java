package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
This class was created by Botosan Octavian on January 14, 2021.
This is a subsystem for the ring intake that we use.
 */

public class IntakeSubsystem extends SubsystemBase {

    private final Motor intakeMotor;

    public IntakeSubsystem(final HardwareMap hMap, final String name) {
        intakeMotor = hMap.get(Motor.class, name);
    }

    /**
     * Start ring intake.
     */
    public void intakeRing() {
        if (intakeMotor.get() != 1.0)
            intakeMotor.set(1.0);
        else if (intakeMotor.get() == 1.0)
            intakeMotor.set(0);
    }

    /**
     * Releases rings from intake.
     */
    public void releaseRing() {
        if (intakeMotor.get() != -1.0)
            intakeMotor.set(-1.0);
        else if (intakeMotor.get() == -1.0)
            intakeMotor.set(0);
    }
}