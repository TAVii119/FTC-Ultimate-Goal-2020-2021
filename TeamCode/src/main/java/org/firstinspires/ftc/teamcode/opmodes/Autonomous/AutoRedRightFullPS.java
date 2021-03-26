package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Autonomous(name = "AutoRedRightFullPS", group = "Red Auto")
public class AutoRedRightFullPS extends LinearOpMode {
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
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

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

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(900,50);

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
    private void caseA(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-1.7, -18.0))
//                .addDisplacementMarker(1, () -> {
//                    // This marker runs 1 inch into the trajectory
//                    setShooterPower(slowShooterSpeed, poweshotRampPos);
//                    liftRingHolder();
//                })
//                .addDisplacementMarker(() -> {
//                    // This marker runs after lineTo()
//
//                })
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(0.0, -61.5, Math.toRadians(-25)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(-33.2, -45.0, Math.toRadians(180)))
                .addDisplacementMarker(7, () -> {
                    // This marker runs 7 inch into the trajectory
//                    placeWobbleGoal(450, 0.24);
                })
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-3.5, -53.0, Math.toRadians(-48)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineTo(new Vector2d(5.0, -25.0))
                .build();

        setShooterPower(0.67, 0.0715);
        drive.followTrajectory(traj1);
        flicker();
        drive.turn(Math.toRadians(8.3));
        sleep(500);
        flicker();
        drive.turn(Math.toRadians(7.5));
        sleep(500);
        flicker();
        setShooterPower(0, 0.045);
        drive.followTrajectory(traj2);
//        placeWobbleGoal(470, 0.24);
        returnWobbleArm();
        drive.followTrajectory(traj3);
//        pickWobbleGoal(100, 0.4);
//        pickWobbleGoal(0, 0.2);
        drive.followTrajectory(traj4);
//        placeWobbleGoal(470, 0.24);
        returnWobbleArm();
        drive.followTrajectory(traj5);
    }

    private void caseB(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-1.7, -18.0))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(18.0, -23.0, Math.toRadians(-60)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(-14.0, -34.0, Math.toRadians(-180)))
                .addDisplacementMarker(7, () -> {
                    // This marker runs 7 inch into the trajectory
//                    placeWobbleGoal(450, 0.24);
                })
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-32.0, -50.0, Math.toRadians(-180)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(-2.0, -33.2))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(25.0, -54.0))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineTo(new Vector2d(5.0, -56))
                .build();

        setShooterPower(0.67, 0.0715);
        drive.followTrajectory(traj1);
        flicker();
        drive.turn(Math.toRadians(8.3));
        sleep(500);
        flicker();
        drive.turn(Math.toRadians(7.5));
        sleep(500);
        flicker();
        setShooterPower(0, 0.025);
        drive.followTrajectory(traj2);
//        placeWobbleGoal(490, 0.24);
        returnWobbleArm();
        // Porneste intake
        intakeRings(1);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        // Oprim intake dupa ce am luat inelul. Dupa traj4 luam wobble
//        pickWobbleGoal(100, 0.4);
//        pickWobbleGoal(0, 0.2);
        setShooterPower(1, 0.062);
        drive.followTrajectory(traj5);
        // Trage
        intakeRings(0);
        sleep(500);
        sleep(700);
        flicker();
        sleep(100);
        flicker();
        setShooterPower(0, 0);
        drive.followTrajectory(traj6);
//        placeWobbleGoal(490, 0.24);
        returnWobbleArm();
        drive.followTrajectory(traj7);
    }

    private void caseC(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-1.7, -18.0))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(52.5, -49.5, Math.toRadians(-75.0)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(-21.0, -32.0, Math.toRadians(-180)))
                .addDisplacementMarker(15, () -> {
                    // This marker runs 7 inch into the trajectory
//                    placeWobbleGoal(450, 0.24);
                })
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-32.0, -50.0, Math.toRadians(-180)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(-3.5, -32.5))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(39.0, -51.5, Math.toRadians(-60.0)))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(4.0, -51.5, Math.toRadians(0)))
                .build();
        setShooterPower(0.67, 0.0715);
        drive.followTrajectory(traj1);
        flicker();
        drive.turn(Math.toRadians(8.3));
        sleep(100);
        flicker();
        drive.turn(Math.toRadians(7.5));
        sleep(100);
        flicker();
        setShooterPower(0, 0.025);
        drive.followTrajectory(traj2);
//        placeWobbleGoal(490, 0.24);
        returnWobbleArm();
        // Porneste intake
        intakeRings(1);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        // Oprim intake dupa ce am luat inelul. Dupa traj4 luam wobble
//        pickWobbleGoal(100, 0.4);
//        pickWobbleGoal(0, 0.2);
        setShooterPower(1, 0.064);
        drive.followTrajectory(traj5);
        // Trage
        intakeRings(0);
        sleep(700);
        flicker();
        sleep(75);
        flicker();
        sleep(75);
        flicker();
        setShooterPower(0, 0);
        drive.followTrajectory(traj6);
//        placeWobbleGoal(490, 0.24);
//        returnWobbleArm();
        drive.followTrajectory(traj7);
    }

    private void intakeRings(double power) {
        map.intakeMotor.setPower(power);
    }

    private void returnWobbleArm() {
        map.wobbleServo.setPosition(0.0);
    }

    private void placeWobbleGoal() {
        map.wobbleServo.setPosition(0.32);
    }

    private void grabWobbleGoal(){
        map.wobbleServoGrabber.setPosition(0.0);
        sleep(200);
    }

    private void ungrabWobbleGoal(){
        map.wobbleServoGrabber.setPosition(0.49);
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
        sleep(400);
    }

    private void dropIntake() {
        double intakeLatch = 0.0, intakeUnlatch = 0.16;
        map.intakeServo.setPosition(intakeUnlatch);
    }

    private void resetServos() {
        map.wobbleServo.setPosition(0.0);
        map.wobbleServoGrabber.setPosition(0.0);
        map.feederServo.setPosition(0.0);
        map.intakeServo.setPosition(0.0);
        map.shooterServo.setPosition(0.0);
    }
}
