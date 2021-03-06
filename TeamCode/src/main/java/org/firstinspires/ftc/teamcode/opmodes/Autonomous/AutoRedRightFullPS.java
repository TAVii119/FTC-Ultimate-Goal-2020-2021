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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

@Autonomous(name = "AutoRedRightFullPS", group = "Red Auto")
public class AutoRedRightFullPS extends LinearOpMode {
    OpenCvCamera webcam;
    RingsDeterminationPipeline pipeline;
    SimpleHardware map = new SimpleHardware();
    private double slowShooterSpeed = 0.83;
//    private double slowShooterSpeed = 0.785;
    private double poweshotRampPos = 0.03;

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

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

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

//             Don't burn CPU cycles busy-looping in this sample
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
        Telemetry telemetry;
        /*
         * Ring position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(830,400);

        static final int REGION_WIDTH = 160;
        static final int REGION_HEIGHT = 170;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
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

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

//    val builder1 = TrajectoryBuilder(startPose, startPose.heading, combinedConstraints)
//            .lineTo(Vector2d(-0.5, -18.0))
//    val builder2 = TrajectoryBuilder(Pose2d(-0.5, -18.0), startPose.heading, combinedConstraints)
//            .strafeTo(Vector2d(4.0, -60.0))
//    val builder3 = TrajectoryBuilder(Pose2d(4.0, -60.0), startPose.heading, combinedConstraints)
//            .lineToLinearHeading(Pose2d(-35.0, -40.0, (-180.0).toRadians))

    Pose2d startPose = new Pose2d(-63, -19.5, Math.toRadians(0.0));
    private void caseA(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-1.7, -20.0))
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
                .lineToSplineHeading(new Pose2d(-33.2, -46.5, Math.toRadians(180)))
                .addDisplacementMarker(7, () -> {
                    // This marker runs 7 inch into the trajectory
                    placeWobbleGoal(450, 0.24);
                })
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-3.5, -55.0, Math.toRadians(-48)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineTo(new Vector2d(5.0, -25.0))
                .build();

        setShooterPower(0.67, 0.045);
        liftRingHolder();
        drive.followTrajectory(traj1);
        flicker();
        drive.turn(Math.toRadians(8));
        flicker();
        drive.turn(Math.toRadians(6.5));
        flicker();
        returnRingHolder();
        setShooterPower(0, 0.045);
        drive.followTrajectory(traj2);
        placeWobbleGoal(470, 0.24);
        returnWobbleArm();
        drive.followTrajectory(traj3);
        pickWobbleGoal(100, 0.4);
        pickWobbleGoal(0, 0.2);
        drive.followTrajectory(traj4);
        placeWobbleGoal(470, 0.24);
        returnWobbleArm();
        drive.followTrajectory(traj5);
    }

    private void caseB(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-1.7, -19.0))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(20.0, -26.0, Math.toRadians(-60)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(-14.0, -34.0, Math.toRadians(-180)))
                .addDisplacementMarker(7, () -> {
                    // This marker runs 7 inch into the trajectory
                    placeWobbleGoal(450, 0.24);
                })
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-32.0, -50.0, Math.toRadians(-180)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(-2.0, -36.5))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(25.0, -57.0))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineTo(new Vector2d(5.0, -56))
                .build();
        setShooterPower(0.63, 0.048);
        liftRingHolder();
        drive.followTrajectory(traj1);
        flicker();
        drive.turn(Math.toRadians(8));
        flicker();
        drive.turn(Math.toRadians(7));
        flicker();
        returnRingHolder();
        setShooterPower(0, 0.045);
        drive.followTrajectory(traj2);
        placeWobbleGoal(470, 0.24);
        returnWobbleArm();
        // Porneste intake
        intakeRings(1);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        // Oprim intake dupa ce am luat inelul. Dupa traj4 luam wobble
        pickWobbleGoal(100, 0.4);
        pickWobbleGoal(0, 0.2);
        setShooterPower(0.9, 0.04);
        drive.followTrajectory(traj5);
        // Trage
        intakeRings(0);
        sleep(500);
        liftRingHolder();
        sleep(700);
        flicker();
        sleep(100);
        flicker();
        setShooterPower(0, 0);
        returnRingHolder();
        drive.followTrajectory(traj6);
        placeWobbleGoal(470, 0.24);
        returnWobbleArm();
        drive.followTrajectory(traj7);
    }

    private void caseC(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-36.0, -37.0))
                .build();
        // 1) Trage primele 2 inele
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(
                        new Vector2d(-18.0, -37.0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(28, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(25))
                .build();
        // 2) Ia urmatoarele 2 inele
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(7.0, -37.0))
                .build();
        // 3) Ia urmatoarele 2 inele
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(-7.0, -12.5))
                .build();
        // 4) Merge la powershot-uri si le trage
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(42.0, -49.5, Math.toRadians(-60.0)))
                .build();
        // 5) Lasa primul wobble goal
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(12.0, -40.0, Math.toRadians(-40.0)))
                .build();
        // 6) Parcheaza

        setShooterPower(0.9, 0.022);
        drive.followTrajectory(traj1);
        // 1) Trage primele 2 inele
        liftRingHolder();
        sleep(700);
        flicker();
        sleep(100);
        flicker();
        sleep(100);
        flicker();
        setShooterPower(0.9, 0.024);
        returnRingHolder();
        intakeRings(1);
        drive.followTrajectory(traj2);
        // 2) Ia urmatoarele 2 inele
        sleep(3000);
        liftRingHolder();
        sleep(500);
        flicker();
        sleep(100);
        setShooterPower(0, 0.033);
        returnRingHolder();
        drive.followTrajectory(traj3);
        // 3) Ia urmatoarele 2 inele
        sleep(3000);
        setShooterPower(0.63, 0.051);
        liftRingHolder();
        intakeRings(0);
        drive.followTrajectory(traj4);
        // 4) Merge la powershot-uri si le trage
        flicker();
        drive.turn(Math.toRadians(7.9));
        flicker();
        drive.turn(Math.toRadians(7.2));
        flicker();
        returnRingHolder();
        setShooterPower(0, 0.045);
        drive.followTrajectory(traj5);
        // 5) Lasa primul wobble goal
        placeWobbleGoal(490, 0.24);
        returnWobbleArm();
        drive.followTrajectory(traj6);
        // 6) Parcheaza
    }

    private void intakeRings(double power) {
        map.intakeMotor.setPower(power);
    }

    private void placeWobbleGoal(int motorPosition, double motorPower) {
        double wobbleGrabberGrab = 0.0, wobbleGrabberUngrab = 0.3;

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
            map.wobbleServo2.setPosition(wobbleGrabberUngrab);
//            sleep(300);
        }
    }

    private void pickWobbleGoal(int motorPosition, double motorPower){
        double wobbleGrabberGrab = 0.0, wobbleGrabberUngrab = 0.3;
        if (opModeIsActive()) {
            map.wobbleServo.setPosition(wobbleGrabberGrab);
            map.wobbleServo2.setPosition(wobbleGrabberGrab);
            sleep(500);
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

    private void setShooterPower(double power, double servoPosition) {
        map.shooterFrontMotor.setPower(power);
        map.shooterBackMotor.setPower(power);
        map.shooterServo.setPosition(servoPosition);
    }

    private void liftRingHolder() {
        double loaderPosLoad = 0.0, loaderPosShoot = 0.215;
        map.loaderFrontServo.setPosition(loaderPosShoot);
        map.loaderBackServo.setPosition(loaderPosShoot);
    }

    private void returnRingHolder() {
        double loaderPosLoad = 0.0;
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
        double intakeLatch = 0.0, intakeUnlatch = 0.12;
        map.intakeServo.setPosition(intakeUnlatch);
    }

    private void resetServos() {
        map.wobbleServo.setPosition(0.0);
        map.wobbleServo2.setPosition(0.0);
        map.loaderFrontServo.setPosition(0.0);
        map.loaderBackServo.setPosition(0.0);
        map.feederServo.setPosition(0.0);
        map.intakeServo.setPosition(0.0);
        map.shooterServo.setPosition(0.0);
        map.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        map.ringBlockerLeft.setPosition(0.031);
        map.ringBlockerRight.setPosition(0.031);
    }
}
