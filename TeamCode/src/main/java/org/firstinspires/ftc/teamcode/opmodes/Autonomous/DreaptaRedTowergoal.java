package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.SimpleHardware;

@Autonomous(name = "DreaptaRedTowergoal", group = "Red Auto")
public class DreaptaRedTowergoal extends LinearOpMode {
    SimpleHardware map = new SimpleHardware();


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        map.init(hardwareMap);

        Pose2d startPose = new Pose2d(-63, -50, Math.toRadians(0.0));

        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-4.5, -59.5), 0)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-4.5, -59.5))
                .strafeTo(new Vector2d(-2, -36))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(-2, -36, Math.toRadians(180)))
                .lineTo(new Vector2d(-20.5, -19))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(-20.5, -19, Math.toRadians(181)))
                .lineTo(new Vector2d(-38, -19))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(-38, -19, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-12.5, -59, Math.toRadians(-10)))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(-12.5, -59, Math.toRadians(0)))
                .strafeLeft(30)
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(new Pose2d(-16, -29, Math.toRadians(0)))
                .forward(23)
                .build();

        resetServos();
        dropIntake();
        drive.followTrajectory(traj1);
        // Lasa wobble goal
        placeWobbleGoal(450, 0.3);
        returnWobbleArm();
        setShooterPower(1, 0.05);
        liftRingHolder();
        drive.followTrajectory(traj2);
        // Trage la tower goal
        sleep(500);
        flicker();
        flicker();
        flicker();
        flicker();
        setShooterPower(0, 0);
        returnRingHolder();
        placeWobbleGoal(450, 0.3);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        // Ia wobble
        pickWobbleGoal(100, 0.5);
        pickWobbleGoal(0, 0.2);
        sleep(300);
        drive.followTrajectory(traj5);
        // Lasa wobble
        placeWobbleGoal(450, 0.3);
        returnWobbleArm();
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        // Parcheaza
    }

    private void placeWobbleGoal(int motorPosition, double motorPower) {
        double wobbleGrabberGrab = 0.0, wobbleGrabberUngrab = 0.2;

        if (opModeIsActive()) {
            map.wobbleMotor.setTargetPosition(motorPosition);
            map.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            map.wobbleMotor.setPower(motorPower);

            while (opModeIsActive() && (map.wobbleMotor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Moving wobble arm: ", map.wobbleMotor.getTargetPosition());
                telemetry.update();
            }

            map.wobbleMotor.setPower(0);
            map.wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            map.wobbleServo.setPosition(wobbleGrabberUngrab);
            sleep(300);
        }
    }

    private void pickWobbleGoal(int motorPosition, double motorPower){
        double wobbleGrabberGrab = 0.0, wobbleGrabberUngrab = 0.2;
        if (opModeIsActive()) {
            map.wobbleServo.setPosition(wobbleGrabberGrab);
            sleep(300);
            map.wobbleMotor.setTargetPosition(motorPosition);
            map.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            map.wobbleMotor.setPower(motorPower);

            while (opModeIsActive() && (map.wobbleMotor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Moving wobble arm: ", map.wobbleMotor.getTargetPosition());
                telemetry.update();
            }

            map.wobbleMotor.setPower(0);
            map.wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void returnWobbleArm() {
        if (opModeIsActive()) {
            map.wobbleMotor.setTargetPosition(0);
            map.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            map.wobbleMotor.setPower(.4);

            while (opModeIsActive() && (map.wobbleMotor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Moving wobble arm: ", map.wobbleMotor.getTargetPosition());
                telemetry.update();
            }

            map.wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            map.wobbleMotor.setPower(0);
        }
    }

    private void setShooterPower(int power, double servoPosition) {
        map.shooterFrontMotor.setPower(power);
        map.shooterBackMotor.setPower(power);
        map.shooterServo.setPosition(servoPosition);
    }

    private void liftRingHolder() {
        double loaderPosLoad = 0.0, loaderPosShoot = 0.21;
        map.loaderFrontServo.setPosition(loaderPosShoot);
        map.loaderBackServo.setPosition(loaderPosShoot);
    }

    private void returnRingHolder() {
        double loaderPosLoad = 0.0, loaderPosShoot = 0.21;
        map.loaderFrontServo.setPosition(loaderPosLoad);
        map.loaderBackServo.setPosition(loaderPosLoad);
    }

    private void flicker() {
        double feederInit = 0.0, feederPush = 0.3;
        map.feederServo.setPosition(feederPush);
        sleep(400);
        map.feederServo.setPosition(feederInit);
        sleep(400);
    }

    private void dropIntake() {
        double intakeLatch = 0.0, intakeUnlatch = 0.3;
        map.intakeServo.setPosition(intakeUnlatch);
    }

    private void resetServos() {
        map.wobbleServo.setPosition(0.0);
        map.loaderFrontServo.setPosition(0.0);
        map.loaderBackServo.setPosition(0.0);
        map.feederServo.setPosition(0.0);
        map.intakeServo.setPosition(0.0);
        map.shooterServo.setPosition(0.0);
        map.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
