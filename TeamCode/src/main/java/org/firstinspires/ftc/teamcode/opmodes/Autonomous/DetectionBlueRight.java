package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
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

@Autonomous(name = "DetectionBlueRight", group = "XDetection Test")
public class DetectionBlueRight extends LinearOpMode {
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

//            if (pipeline.position == RingsDeterminationPipeline.RingPosition.NONE) {
//                webcam.stopStreaming();
//                webcam.stopRecordingPipeline();
//                caseC(drive);
//                sleep(30000);
//            } else if (pipeline.position == RingsDeterminationPipeline.RingPosition.ONE) {
//                webcam.stopStreaming();
//                webcam.stopRecordingPipeline();
//                caseC(drive);
//                sleep(30000);
//            } else if (pipeline.position == RingsDeterminationPipeline.RingPosition.FOUR) {
//                webcam.stopStreaming();
//                webcam.stopRecordingPipeline();
//                caseC(drive);
//                sleep(30000);
//            } else {
//                webcam.stopStreaming();
//                webcam.stopRecordingPipeline();
//                caseC(drive);
//                sleep(30000);
//            }

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

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(120,300);

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

//    Pose2d startPose = new Pose2d(-63.0, 27.3, Math.toRadians(0.0)); // RIGHT
    Pose2d startPose = new Pose2d(-63.0, 51.0, Math.toRadians(0.0)); // LEFT
    private void caseA(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-30.0, 52.0))
                .addDisplacementMarker(7, () -> {
                    // This marker runs 7 inch into the trajectory
//                    placeWobbleGoal();
                })
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(1.0, 48.0))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(10.0, 38.0))
                .build();

        rotateTurret(0.15);
        setShooterPower(0.7, 0.273);
        drive.followTrajectory(traj1);
        sleep(3300);
        flicker();
        sleep(1100);
        flicker();
        sleep(2800);
        flicker();
        sleep(800);
        setShooterPower(0, 0.273);
        sleep(10000);
        drive.followTrajectory(traj2);
        placeWobbleGoal();
        sleep(200);
        drive.followTrajectory(traj3);

        map.wobbleServoLeft.setPosition(0.0);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    private void caseB(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-25.0, 33.0))
                .addDisplacementMarker(7, () -> {
                    // This marker runs 7 inch into the trajectory
//                    placeWobbleGoal();
                })
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-19.0, 33.0))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(14.0, 43.0, Math.toRadians(-90)))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(10.0, 38.0))
                .build();

        sleep(5000);
        rotateTurret(0.21);
        setShooterPower(0.7, 0.273);
        drive.followTrajectory(traj1);
        sleep(3300);
        flicker();
        sleep(900);
        flicker();
        sleep(1800);
        flicker();
        sleep(800);
        setShooterPower(0, 0.273);
        intakeRings(1);
        setShooterPower(0.7, 0.28);
        sleep(1000);
        drive.followTrajectory(traj2);
        sleep(1200);
        intakeRings(0);
        flicker();
        sleep(500);
        flicker();
        sleep(800);
        setShooterPower(0, 0.28);
        drive.followTrajectory(traj3);
        sleep(200);
        placeWobbleGoal();
        drive.followTrajectory(traj4);

        map.wobbleServoLeft.setPosition(0.0);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    private void caseC(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-17.0, 33),
                new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                        )
                ),
                new ProfileAccelerationConstraint(30))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-15.0, 33),
                new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                        )
                ),
                new ProfileAccelerationConstraint(10))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-5, 34),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(10))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(47.5, 52.0))
                .build(); // duce wobble
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineTo(new Vector2d(13, 38.0))
                .build();

        rotateTurret(0.22);
        setShooterPower(0.7, 0.275);
        drive.followTrajectory(traj1);
        sleep(1500);
        flicker();
        sleep(700);
        flicker();
        sleep(800);
        flicker();
        sleep(400);
        intakeRings(1);
        drive.followTrajectory(traj2);
        setShooterPower(0.7, 0.275);
        sleep(4000);
        flicker();
        sleep(200);
        setShooterPower(0.7, 0.283);
        drive.followTrajectory(traj3);
        sleep(1000);
        intakeRings(0);
        rotateTurret(0.21);
        sleep(400);
        flicker();
        sleep(800);
        flicker();
        sleep(1300);
        flicker();
        sleep(700);
        setShooterPower(0, 0.283);
        drive.followTrajectory(traj4);
        placeWobbleGoal();
        sleep(200);
        drive.followTrajectory(traj5);

        map.wobbleServoLeft.setPosition(0.0);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    private void intakeRings(double power) {
        map.intakeMotor.setPower(power);
        map.intakeMotor2.setPower(power);
    }

    private void placeWobbleGoal(){
        map.wobbleServoLeft.setPosition(0.3);
        sleep(200);
    }

    private void setShooterPower(double power, double servoPosition) {
        map.shooterFrontMotor.setPower(power);
        map.shooterServo.setPosition(servoPosition);
    }

    private void flicker() {
        double feederInit = 0.0, feederPush = 0.15;
        map.feederServo.setPosition(feederPush);
        sleep(400);
        map.feederServo.setPosition(feederInit);
        sleep(200);
    }

    private void rotateTurret(double position) {
        map.turretServo.setPosition(position);
    }

    private void resetServos() {
//        map.wobbleServoLeft.setPosition(0.0);
//        map.feederServo.setPosition(0.0);
//        map.shooterServo.setPosition(0.0);
//        map.turretServo.setPosition(0.21);
    }
}
