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

@Autonomous(name = "AutoRedRightFullPS", group = "Red Auto")
public class AutoRedRightFullPS extends LinearOpMode {
    OpenCvCamera webcam;
    RingsDeterminationPipeline pipeline;
    SimpleHardware map = new SimpleHardware();
    com.arcrobotics.ftclib.geometry.Pose2d startingPose = new com.arcrobotics.ftclib.geometry.Pose2d(20, -10, new Rotation2d());
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private static T265Camera slamra = null;

    @Override
    public void runOpMode()
    {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
        slamra.setPose(startingPose);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        map.init(hardwareMap);
//        resetServos();


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
        slamra.start();
        while (opModeIsActive())
        {
            final int robotRadius = 9; // inches

            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
            if (up == null) return;

            // We divide by 0.0254 to convert meters to inches
            Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
            Rotation2d rotation = up.pose.getRotation();

            field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
            double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
            double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
            double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
            field.strokeLine(x1, y1, x2, y2);

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

//            dropIntake();
//            if (pipeline.position == RingsDeterminationPipeline.RingPosition.NONE) {
//                webcam.stopStreaming();
//                webcam.stopRecordingPipeline();
////                caseA(drive);
//                sleep(30000);
//            } else if (pipeline.position == RingsDeterminationPipeline.RingPosition.ONE) {
//                webcam.stopStreaming();
//                webcam.stopRecordingPipeline();
////                caseB(drive);
//                sleep(30000);
//            } else if (pipeline.position == RingsDeterminationPipeline.RingPosition.FOUR) {
//                webcam.stopStreaming();
//                webcam.stopRecordingPipeline();
////                caseC(drive);
//                sleep(30000);
//            } else {
//                webcam.stopStreaming();
//                webcam.stopRecordingPipeline();
////                caseA(drive);
//                sleep(30000);
//            }

        }
        slamra.stop();
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

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(1050,320);

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

    Pose2d startPose = new Pose2d(-63, -19.5, Math.toRadians(0.0));
//    private void caseA(SampleMecanumDrive drive) {
//        drive.setPoseEstimate(startPose);
//
//        Trajectory traj1 = drive.trajectoryBuilder(startPose)
//                .lineTo(new Vector2d(-3.0, -22.0))
//                .addDisplacementMarker(7, () -> {
//                    // This marker runs 7 inch into the trajectory
////                    placeWobbleGoal();
//                })
//                .build();
//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .lineTo(new Vector2d(22.0, -50.0))
//                .build();
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                .lineToSplineHeading(new Pose2d(-17.0, -30.0, Math.toRadians(-90)))
//                .build();
//
//        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .lineToSplineHeading(new Pose2d(-38.0, -60.0, Math.toRadians(-90)))
//                .build();
//
//        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(-38.0, -60.0, Math.toRadians(0)))
//                .lineToSplineHeading(new Pose2d(1.5, -47.0, Math.toRadians(50)))
//                .build();
//
//        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
//                .lineToSplineHeading(new Pose2d(10.0, -25.0, Math.toRadians(0)))
//                .build();
//
//        setShooterPower(0.57, 0.034);
//        drive.followTrajectory(traj1);
//        sleep(200);
//        flicker();
//        rotateTurret(0.24);
//        sleep(200);
//        flicker();
//        rotateTurret(0.26);
//        sleep(200);
//        flicker();
//        sleep(200);
//        setShooterPower(0, 0.03);
//        drive.followTrajectory(traj2);
//        placeWobbleGoal();
//        ungrabWobbleGoal();
//        drive.followTrajectory(traj3);
//        drive.followTrajectory(traj4);
//        grabWobbleGoal();
//        sleep(500);
//        returnWobbleArm();
//        sleep(500);
//        drive.followTrajectory(traj5);
//        placeWobbleGoal();
//        ungrabWobbleGoal();
//        sleep(200);
//        map.wobbleServo.setPosition(0);
//        drive.followTrajectory(traj6);
//    }
//
//    private void caseB(SampleMecanumDrive drive) {
//        drive.setPoseEstimate(startPose);
//
//        Trajectory traj1 = drive.trajectoryBuilder(startPose)
//                .lineTo(new Vector2d(-3.0, -22.0))
//                .addDisplacementMarker(7, () -> {
//                    // This marker runs 7 inch into the trajectory
////                    placeWobbleGoal();
//                })
//                .build();
//
//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .lineTo(new Vector2d(45.0, -24.0))
//                .build();
//
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                .lineToSplineHeading(new Pose2d(-28.0, -40.0, Math.toRadians(180)))
//                .build();
//
//        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .lineToSplineHeading(new Pose2d(-38.0, -50.0, Math.toRadians(-90)))
//                .build();
//
//        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(-38.0, -50.0, Math.toRadians(0)))
//                .lineToSplineHeading(new Pose2d(-3.5, -32.5, Math.toRadians(-10)))
//                .build();
//
//        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
//                .lineToSplineHeading(new Pose2d(24.0, -29.0, Math.toRadians(40)))
//                .build();
//
//        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
//                .lineToSplineHeading(new Pose2d(10.0, -25.0, Math.toRadians(0)))
//                .build();
//
//        setShooterPower(0.57, 0.034);
//        drive.followTrajectory(traj1);
//        sleep(200);
//        flicker();
//        rotateTurret(0.24);
//        sleep(200);
//        flicker();
//        rotateTurret(0.26);
//        sleep(200);
//        flicker();
//        sleep(200);
//        setShooterPower(0, 0.034);
//        drive.followTrajectory(traj2);
//        placeWobbleGoal();
//        ungrabWobbleGoal();
//        intakeRings(1);
//        drive.followTrajectory(traj3);
//        drive.followTrajectory(traj4);
//        grabWobbleGoal();
//        sleep(500);
//        intakeRings(0);
//        returnWobbleArm();
//        sleep(500);
//        rotateTurret(0.23);
//        setShooterPower(0.7, 0.045);
//        drive.followTrajectory(traj5);
//        sleep(200);
//        flicker();
//        sleep(400);
//        setShooterPower(0.0, 0.045);
//        drive.followTrajectory(traj6);
//        placeWobbleGoal();
//        ungrabWobbleGoal();
//        sleep(200);
//        map.wobbleServo.setPosition(0.0);
//        drive.followTrajectory(traj7);
//    }
//
//    private void caseC(SampleMecanumDrive drive) {
//        drive.setPoseEstimate(startPose);
//
//        Trajectory traj1 = drive.trajectoryBuilder(startPose)
//                .lineTo(new Vector2d(-26, -38))
//                .build();
//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .lineTo(new Vector2d(-25, -38),
//                new MinVelocityConstraint(
//                        Arrays.asList(
//                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                new MecanumVelocityConstraint(35, DriveConstants.TRACK_WIDTH)
//                        )
//                ),
//                new ProfileAccelerationConstraint(35))
//                .build();
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                .lineTo(new Vector2d(-5, -38),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(35, DriveConstants.TRACK_WIDTH)
//                                )
//                        ),
//                        new ProfileAccelerationConstraint(35))
//                .build();
//        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .lineTo(new Vector2d(-6.0, -22.0))
//                .build();
//        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
//                .lineToSplineHeading(new Pose2d(56.0, -45.0, Math.toRadians(20)))
//                .build();
//        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
//                .lineToSplineHeading(new Pose2d(10.0, -25.0, Math.toRadians(0)))
//                .build();
//        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
//                .lineToSplineHeading(new Pose2d(-28.0, -40.0, Math.toRadians(180)))
//                .build();
//
//        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
//                .lineToSplineHeading(new Pose2d(-38.0, -50.0, Math.toRadians(-90)))
//                .build();

//        setShooterPower(0.7, 0.028);
//        drive.followTrajectory(traj1);
//        sleep(200);
//        flicker();
//        sleep(100);
//        flicker();
//        sleep(100);
//        flicker();
//        sleep(400);
//        intakeRings(1);
//        drive.followTrajectory(traj2);
//        sleep(1500);
//        flicker();
//        flicker();
//        sleep(200);
//        setShooterPower(0.57, 0.034);
//        drive.followTrajectory(traj3);
//        drive.followTrajectory(traj4);
//        sleep(200);
//        flicker();
//        intakeRings(0);
//        rotateTurret(0.245);
//        sleep(200);
//        flicker();
//        rotateTurret(0.265);
//        sleep(200);
//        flicker();
//        sleep(200);
//        setShooterPower(0, 0.03);
//        drive.followTrajectory(traj5);
//        placeWobbleGoal();
//        ungrabWobbleGoal();
//        drive.followTrajectory(traj6);
//        map.wobbleServo.setPosition(0);
////        drive.followTrajectory(traj6);
////        drive.followTrajectory(traj7);
////        grabWobbleGoal();
////        sleep(200);
////        returnWobbleArm();
//
//    }

    private void intakeRings(double power) {
        map.intakeMotor.setPower(power);
    }

//    private void returnWobbleArm() {
//        map.wobbleServo.setPosition(0.22);
//    }
//
//    private void placeWobbleGoal() {
//        map.wobbleServo.setPosition(0.28);
//    }
//
//    private void grabWobbleGoal(){
//        map.wobbleServoGrabber.setPosition(0.0);
//        sleep(200);
//    }
//
//    private void ungrabWobbleGoal(){
//        map.wobbleServoGrabber.setPosition(0.49);
//        sleep(200);
//    }

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
        sleep(400);
    }

    private void dropIntake() {
//        double intakeLatch = 0.0, intakeUnlatch = 0.16;
        map.intakeServo.setPosition(0.6);
    }

    private void rotateTurret(double position) {
        map.turretServo.setPosition(position);
    }

    private void resetServos() {
//        map.wobbleServo.setPosition(0.0);
//        map.wobbleServoGrabber.setPosition(0.0);
//        map.feederServo.setPosition(0.0);
//        map.intakeServo.setPosition(0.0);
//        map.shooterServo.setPosition(0.0);
//        map.turretServo.setPosition(0.23);
    }
}
