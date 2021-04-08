package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.SimpleHardware;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

@Autonomous(name = "RedLeftPsOnly", group = "Red Auto")
public class RedLeftPsOnly extends LinearOpMode {
    OpenCvCamera webcam;
    RingsDeterminationPipeline pipeline;
    SimpleHardware map = new SimpleHardware();

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        map.init(hardwareMap);
        resetServos();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RingsDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            final int robotRadius = 9; // inches

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

//            blockersUp();
            dropIntake();
            if (pipeline.position == RingsDeterminationPipeline.RingPosition.NONE) {
                webcam.stopStreaming();
                webcam.stopRecordingPipeline();
                caseA(drive);
                sleep(30000);
            } else if (pipeline.position == RingsDeterminationPipeline.RingPosition.ONE) {
                webcam.stopStreaming();
                webcam.stopRecordingPipeline();
                caseB(drive);
                sleep(30000);
            } else if (pipeline.position == RingsDeterminationPipeline.RingPosition.FOUR) {
                webcam.stopStreaming();
                webcam.stopRecordingPipeline();
                caseC(drive);
                sleep(30000);
            } else {
                webcam.stopStreaming();
                webcam.stopRecordingPipeline();
                caseA(drive);
                sleep(30000);
            }

        }
    }


    public static class RingsDeterminationPipeline extends OpenCvPipeline
    {
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(1050,280);

        static final int REGION_WIDTH = 160;
        static final int REGION_HEIGHT = 170;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        private volatile RingPosition position = RingPosition.FOUR;

        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);
            avg1 = (int) Core.mean(region1_Cb).val[0];
            Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE, 2);

            position = RingPosition.FOUR;
            if(avg1 > FOUR_RING_THRESHOLD) {
                position = RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = RingPosition.ONE;
            } else {
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(input, region1_pointA, region1_pointB, GREEN, -1);

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

    Pose2d startPose = new Pose2d(-63.0, -24.3, Math.toRadians(0.0));
    private void caseA(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-6.0, -38.0))
                .addDisplacementMarker(7, () -> {
                    // This marker runs 7 inch into the trajectory
//                    placeWobbleGoal();
                })
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(22.0, -46.0))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(5, -10.0))
                .build();

        setShooterPower(0.525, 0.14);
        drive.followTrajectory(traj1);
        sleep(300);
        flicker();
        rotateTurret(0.26);
        sleep(300);
        flicker();
        rotateTurret(0.27);
        sleep(300);
        flicker();
        sleep(200);
        setShooterPower(0, 0.03);
        drive.followTrajectory(traj2);
        placeWobbleGoal();
        ungrabWobbleGoal();
        sleep(200);
        returnWobbleArm();
        sleep(200);
        grabWobbleGoal();
        drive.followTrajectory(traj3);
    }

    private void caseB(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-3.0, -22.0))
                .addDisplacementMarker(7, () -> {
                    // This marker runs 7 inch into the trajectory
//                    placeWobbleGoal();
                })
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(42.5, -19.0))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(5, -10.0))
                .build();

        setShooterPower(0.525, 0.14);
        drive.followTrajectory(traj1);
        sleep(300);
        flicker();
        rotateTurret(0.26);
        sleep(300);
        flicker();
        rotateTurret(0.27);
        sleep(300);
        flicker();
        sleep(200);
        setShooterPower(0, 0.03);
        drive.followTrajectory(traj2);
        placeWobbleGoal();
        ungrabWobbleGoal();
        sleep(200);
        returnWobbleArm();
        sleep(200);
        grabWobbleGoal();
        drive.followTrajectory(traj3);
    }

    private void caseC(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-3.0, -22.0))
                .addDisplacementMarker(7, () -> {
                    // This marker runs 7 inch into the trajectory
//                    placeWobbleGoal();
                })
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(53.0, -46.7, Math.toRadians(30)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(5, -10.0, Math.toRadians(0.0)))
                .build();

        setShooterPower(0.525, 0.14);
        drive.followTrajectory(traj1);
        sleep(300);
        flicker();
        rotateTurret(0.26);
        sleep(300);
        flicker();
        rotateTurret(0.27);
        sleep(300);
        flicker();
        sleep(200);
        setShooterPower(0, 0.03);
        drive.followTrajectory(traj2);
        placeWobbleGoal();
        ungrabWobbleGoal();
        sleep(200);
        returnWobbleArm();
        sleep(200);
        grabWobbleGoal();
        drive.followTrajectory(traj3);
    }

    private void intakeRings(double power) {
        map.intakeMotor.setPower(power);
    }

    private void returnWobbleArm() {
        map.wobbleServoRight.setPosition(0.0);
    }

    private void placeWobbleGoal() {
        map.wobbleServoRight.setPosition(0.28);
    }

    private void grabWobbleGoal(){
        map.wobbleServoGrabberRight.setPosition(0.0);
        sleep(200);
    }

    private void ungrabWobbleGoal(){
        map.wobbleServoGrabberRight.setPosition(0.49);
        sleep(200);
    }

    private void setShooterPower(double power, double servoPosition) {
        map.shooterFrontMotor.setPower(power);
        map.shooterBackMotor.setPower(power);
        map.shooterServo.setPosition(servoPosition);
    }

    private void flicker() {
        double feederInit = 0.0, feederPush = 0.3;
        map.feederServo.setPosition(feederPush);
        sleep(400);
        map.feederServo.setPosition(feederInit);
        sleep(200);
    }

    private void dropIntake() { map.intakeServo.setPosition(0.36); }

    private void blockersUp() {
        map.ringBlockerLeft.setPosition(0.42);
        map.ringBlockerRight.setPosition(0.42);
    }

    private void rotateTurret(double position) {
        map.turretServo.setPosition(position);
    }

    private void resetServos() {
        map.wobbleServoRight.setPosition(0.0);
        map.wobbleServoGrabberRight.setPosition(0.0);
        map.feederServo.setPosition(0.0);
        map.intakeServo.setPosition(0.0);
        map.shooterServo.setPosition(0.0);
        map.turretServo.setPosition(0.23);
        map.ringBlockerRight.setPosition(0.0);
        map.ringBlockerLeft.setPosition(0.0);
    }
}
