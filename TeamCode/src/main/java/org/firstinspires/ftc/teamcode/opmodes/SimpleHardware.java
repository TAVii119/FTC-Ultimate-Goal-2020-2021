package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * HardwareTest for team Delta Force
 * Created on 21.11.2020 by Botosan Octavian
 */

public class SimpleHardware {
    // INSTANTIATE MOTORS
    public DcMotor flMotor = null; // FRONT LEFT CHASSIS MOTOR
    public DcMotor frMotor = null; // FRONT RIGHT CHASSIS MOTOR
    public DcMotor blMotor = null; // BACK LEFT CHASSIS MOTOR
    public DcMotor brMotor = null; // BACK RIGHT CHASSI MOTOR
    public DcMotor shooterFrontMotor = null; // SHOOTER FRONT MOTOR
    public DcMotor shooterBackMotor = null; // SHOOTER BACK MOTOR
    public DcMotor intakeMotor = null; // INTAKE MOTOR
    public DcMotor wobbleMotor = null; // WOBBLE GOAL MECHANSIM MOTOR

    // INSTANTIATE SERVOS
    public Servo wobbleServo = null;
    public Servo loaderFrontServo = null;
    public Servo loaderBackServo = null;
    public Servo feederServo = null;
    public Servo intakeServo = null;
    public Servo shooterServo = null;

    // CREATE NEW HardwareMap
    HardwareMap robotMap;

    // DEFINE NEW HardwareMap
    public void init(HardwareMap robotMap) {

        // DEFINE MOTORS
        flMotor = robotMap.get(DcMotor.class, "flMotor");
        frMotor = robotMap.get(DcMotor.class, "frMotor");
        blMotor = robotMap.get(DcMotor.class, "blMotor");
        brMotor = robotMap.get(DcMotor.class, "brMotor");
        shooterFrontMotor = robotMap.get(DcMotor.class, "shooterFrontMotor");
        shooterBackMotor = robotMap.get(DcMotor.class, "shooterBackMotor");
        intakeMotor = robotMap.get(DcMotor.class, "intakeMotor");
        wobbleMotor = robotMap.get(DcMotor.class, "wobbleMotor");

        // SET MOTOR DIRECTION
        flMotor.setDirection(DcMotor.Direction.REVERSE);
        frMotor.setDirection(DcMotor.Direction.FORWARD);
        blMotor.setDirection(DcMotor.Direction.REVERSE);
        brMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterBackMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        wobbleMotor.setDirection(DcMotor.Direction.FORWARD);

        // SET MOTOR POWER
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        shooterFrontMotor.setPower(0);
        shooterBackMotor.setPower(0);
        intakeMotor.setPower(0);
        wobbleMotor.setPower(0);

        // SET MOTOR MODE
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // ODOMETRY RIGHT
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // ODOMETRY LEFT
        wobbleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // ODOMETRY STRAFE

        // SET MOTOR ZeroPowerBehavior
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // DEFINE SERVOS
        wobbleServo = robotMap.get(Servo.class, "wobbleServo");
        loaderFrontServo = robotMap.get(Servo.class, "loaderFrontServo");
        loaderBackServo = robotMap.get(Servo.class, "loaderBackServo");
        feederServo = robotMap.get(Servo.class, "feederServo");
        intakeServo = robotMap.get(Servo.class, "intakeServo");
        shooterServo = robotMap.get(Servo.class, "shooterServo");

        // SET SERVO DIRECTION
        wobbleServo.setDirection(Servo.Direction.FORWARD);
        loaderFrontServo.setDirection(Servo.Direction.FORWARD);
        loaderBackServo.setDirection(Servo.Direction.REVERSE);
        feederServo.setDirection(Servo.Direction.FORWARD);
        intakeServo.setDirection(Servo.Direction.FORWARD);
        shooterServo.setDirection(Servo.Direction.REVERSE);

        // SET SERVO POSITION
        wobbleServo.setPosition(0.0);
        loaderFrontServo.setPosition(0.0);
        loaderBackServo.setPosition(0.0);
        feederServo.setPosition(0.0);
        intakeServo.setPosition(0.0);
        shooterServo.setPosition(0.0);
    }
}