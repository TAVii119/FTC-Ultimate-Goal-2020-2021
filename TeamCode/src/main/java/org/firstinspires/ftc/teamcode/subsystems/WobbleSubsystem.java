package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.opmodes.TeleOpSimple.GAMEPAD_LOCKOUT;

public class WobbleSubsystem extends SubsystemBase {

    Servo servo;
    Motor motor;
    Telemetry tele;
    private Deadline gamepadRateLimit;
    private final static int GAMEPAD_LOCKOUT = 200;

    public WobbleSubsystem(Servo pickMeUp, Motor wobbleMotor, Telemetry telemetry){
        servo = pickMeUp;
        motor = wobbleMotor;
        tele = telemetry;

        motor.resetEncoder();
    }

    public void moveWobbleMotor(double spd) {
        motor.set(spd);
    }

    public void moveWobbleServo() {
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (servo.getPosition() == 0.0)
            servo.setPosition(0.5);
        else servo.setPosition(0.0);
        gamepadRateLimit.reset();
    }

    @Override
    public void periodic() {
        tele.addData("Wobble Motor Position: ", motor.getCurrentPosition());
        tele.addData("Wobble Servo Position: ", servo.getPosition());
        tele.update();
    }
}
