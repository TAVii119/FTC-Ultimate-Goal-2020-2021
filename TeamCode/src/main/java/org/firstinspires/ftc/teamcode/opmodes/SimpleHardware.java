package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * HardwareTest for team Delta Force
 * Created on 21.11.2020 by Botosan Octavian
 **/

public class SimpleHardware {
    // INSTANTIATE MOTORS
    public DcMotor flMotor = null; // FRONT LEFT CHASSIS MOTOR
    public DcMotor frMotor = null; // FRONT RIGHT CHASSIS MOTOR
    public DcMotor blMotor = null; // BACK LEFT CHASSIS MOTOR
    public DcMotor brMotor = null; // BACK RIGHT CHASSIS MOTOR
    public DcMotor shooterFrontMotor = null; // SHOOTER FRONT MOTOR
    public DcMotor intakeMotor = null; // INTAKE MOTOR
    public DcMotor intakeMotor2 = null; // INTAKE MOTOR
    public DcMotor wobbleMotor = null;
    // INSTANTIATE SERVOS
//    public Servo loaderFrontServo = null;
//    public Servo loaderBackServo = null;
    public Servo feederServo = null;
    public Servo shooterServo = null;
    public Servo ringBlockerLeft = null;
    public Servo ringBlockerRight = null;
    public Servo turretServo = null;
    public Servo wobbleServo = null;
    public Servo wobbleServoLeft = null;
    public Servo wobbleServoRight = null;
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
        intakeMotor = robotMap.get(DcMotor.class, "intakeMotor");
        intakeMotor2 = robotMap.get(DcMotor.class, "intakeMotor2");
        wobbleMotor = robotMap.get(DcMotor.class, "wobbleMotor");



        // SET MOTOR DIRECTION
        flMotor.setDirection(DcMotor.Direction.REVERSE);
        frMotor.setDirection(DcMotor.Direction.FORWARD);
        blMotor.setDirection(DcMotor.Direction.REVERSE);
        brMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);

        // SET MOTOR POWER
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        shooterFrontMotor.setPower(0);
        intakeMotor.setPower(0);
        intakeMotor2.setPower(0);
        wobbleMotor.setPower(0);

        // SET MOTOR MODE
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // ODOMETRY LEFT
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // ODOMETRY RIGHT
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // ODOMETRY STRAFE
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // SET MOTOR ZeroPowerBehavior
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // DEFINE SERVOS
        feederServo = robotMap.get(Servo.class, "feederServo");
        shooterServo = robotMap.get(Servo.class, "shooterServo");
        ringBlockerLeft = robotMap.get(Servo.class, "ringBlockerLeft");
        ringBlockerRight = robotMap.get(Servo.class, "ringBlockerRight");
        turretServo = robotMap.get(Servo.class, "turretServo");
        wobbleServo = robotMap.get(Servo.class, "wobbleServo");
        wobbleServoLeft = robotMap.get(Servo.class, "wobbleServoLeft");
        wobbleServoRight = robotMap.get(Servo.class, "wobbleServoRight");

        // SET SERVO DIRECTION
        feederServo.setDirection(Servo.Direction.REVERSE);
        shooterServo.setDirection(Servo.Direction.REVERSE);
        ringBlockerRight.setDirection(Servo.Direction.REVERSE);
        turretServo.setDirection(Servo.Direction.FORWARD);
        wobbleServoLeft.setDirection(Servo.Direction.REVERSE);

        // SET SERVO POSITION
//        ringBlockerLeft.setPosition(0.0);
//        ringBlockerRight.setPosition(0.0);
//        feederServo.setPosition(0.0);
//        shooterServo.setPosition(0.0);
//        turretServo.setPosition(0.0);
    }
}