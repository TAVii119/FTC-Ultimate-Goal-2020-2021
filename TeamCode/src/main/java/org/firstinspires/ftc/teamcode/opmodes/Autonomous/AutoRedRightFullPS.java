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
    OpenCvInternalCamera phoneCam;
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
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new RingsDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.addData("Position", pipeline.position);
//            telemetry.update();
//
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(50);

            dropIntake();

            if (pipeline.position == RingsDeterminationPipeline.RingPosition.NONE) {
                phoneCam.stopStreaming();
                phoneCam.stopRecordingPipeline();
                caseA(drive);
//                caseC(drive);
                sleep(30000);
            } else if (pipeline.position == RingsDeterminationPipeline.RingPosition.ONE) {
                phoneCam.stopStreaming();
                phoneCam.stopRecordingPipeline();
                caseB(drive);
                sleep(30000);
            } else if (pipeline.position == RingsDeterminationPipeline.RingPosition.FOUR) {
                phoneCam.stopStreaming();
                phoneCam.stopRecordingPipeline();
                caseC(drive);
                sleep(30000);
            } else {
                phoneCam.stopStreaming();
                phoneCam.stopRecordingPipeline();
                caseA(drive);
                sleep(30000);
            }

        }
    }

    public static class RingsDeterminationPipeline extends OpenCvPipeline
    {
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(200,170);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

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

    Pose2d startPose = new Pose2d(-63, -50, Math.toRadians(0.0));
    private void caseA(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-2, -43.5))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-2, -43.5, Math.toRadians(-40)))
                .lineToSplineHeading(new Pose2d(-3.5, -73.5, Math.toRadians(-50)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(-1.5, -73.5, Math.toRadians(180)))
                .lineTo(new Vector2d(-15, -65))
                .build();

        Trajectory traj4= drive.trajectoryBuilder(new Pose2d(-15, -66.5, Math.toRadians(190)))
                .lineTo(new Vector2d(-38, -65))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(-38, -65))
                .lineToSplineHeading(new Pose2d(-7.5, -73, Math.toRadians(-70)))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(-7.5, -73.0))
                .strafeTo(new Vector2d(12, -50))
                .build();

        setShooterPower(slowShooterSpeed, poweshotRampPos);
        drive.followTrajectory(traj1);
        // Trage la powershot
        liftRingHolder();
        sleep(750);
        flicker();
        // Rotatie
        drive.turn(Math.toRadians(10.5));
        sleep(300);
        flicker();
        // Rotatie
        drive.turn(Math.toRadians(9));
        sleep(300);
        flicker();
        setShooterPower(0, 0);
        drive.turn(Math.toRadians(0));
        returnRingHolder();
        drive.followTrajectory(traj2);
        // Lasa wobble goal
        placeWobbleGoal(450, 0.3);
        sleep(500);
        returnWobbleArm();
        drive.followTrajectory(traj3);
        placeWobbleGoal(450, 0.3);
        drive.followTrajectory(traj4);
        // Ia wobble
        pickWobbleGoal(100, 0.4);
        pickWobbleGoal(0, 0.2);
        sleep(300);
        drive.followTrajectory(traj5);
        // Lasa wobble
        placeWobbleGoal(450, 0.3);
        returnWobbleArm();
        drive.followTrajectory(traj6);
        // Parcheaza
    }

    private void caseB(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-2, -43))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-2, -43, Math.toRadians(-50)))
                .lineToSplineHeading(new Pose2d(19, -50, Math.toRadians(-50)))
                .build();//lasa wobble

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(19, -50, Math.toRadians(180)))
                .lineTo(new Vector2d(-15, -65))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(-15, -65, Math.toRadians(190)))
                .lineTo(new Vector2d(-25, -58.2))
                .build();// ia wobble

        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(-25, -58.2, Math.toRadians(190)))
                .lineTo(new Vector2d(-40.8, -62.2))
                .build();// ia wobble

        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(-40.8, -62.2, Math.toRadians(0)))
                .lineTo(new Vector2d(-4, -57))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(new Pose2d(-4, -57, Math.toRadians(-10)))
                .lineTo(new Vector2d(10, -64))
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(new Pose2d(10, -64))
                .back(3)
                .build();

        setShooterPower(slowShooterSpeed, poweshotRampPos);
        drive.followTrajectory(traj1);
        // Trage la powershot
        liftRingHolder();
        sleep(750);
        flicker();
        // Rotatie
        drive.turn(Math.toRadians(9));
        sleep(300);
        flicker();
        // Rotatie
        drive.turn(Math.toRadians(9));
        sleep(300);
        flicker();
        setShooterPower(0, 0);
        drive.turn(Math.toRadians(0));
        returnRingHolder();
        drive.followTrajectory(traj2);
        // Lasa wobble goal
        placeWobbleGoal(450, 0.3);
        sleep(500);
        returnWobbleArm();
        intakeRings(1);
        drive.followTrajectory(traj3);
        placeWobbleGoal(450, 0.3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        // Ia wobble
        pickWobbleGoal(100, 0.4);
        pickWobbleGoal(0, 0.2);
        intakeRings(0);
        sleep(300);
        setShooterPower(1, 0.04);
        drive.followTrajectory(traj6);
        drive.turn(Math.toRadians(-5));
        // Trage
        liftRingHolder();
        sleep(500);
        flicker();
        sleep(300);
        flicker();
        setShooterPower(0, 0);
        returnRingHolder();
        // Lasa wobble
        drive.turn(Math.toRadians(-10));
        drive.followTrajectory(traj7);
        placeWobbleGoal(450, 0.3);
        returnWobbleArm();
        drive.followTrajectory(traj8);
        // Parcheaza
    }

    private void caseC(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-2, -43.5))
                .build();// Trage la power shot-uri

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-2, -43.5))
                .strafeLeft(12)
                .build(); //Merge in spate

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(-2.5, -36.5))
                .back(45)
                .build(); //Merge in spate

        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(-42.5, -36.5))
                .strafeRight(23)
                .build(); //Merge in spate

        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(-50,  -56.5))
                .forward(
                        32.5,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(35, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(30))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(-17.5, -56.5))
                .forward(15)
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(new Pose2d(-2.5, -60.5))
                .lineToSplineHeading(new Pose2d(38, -85, Math.toRadians(-30)))
                .build();// lasa wobble

        Trajectory traj8 = drive.trajectoryBuilder(new Pose2d(38, -85, Math.toRadians(-30)))
                .lineToConstantHeading(new Vector2d(6,-60))
                .build();

        setShooterPower(slowShooterSpeed, poweshotRampPos);
        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(2));
        // Trage la powershot
        liftRingHolder();
        sleep(750);
        flicker();
        // Rotatie
        drive.turn(Math.toRadians(9));
        sleep(300);
        flicker();
        // Rotatie
        drive.turn(Math.toRadians(9));
        sleep(300);
        flicker();
        setShooterPower(0, 0);
        returnRingHolder();
        drive.turn(Math.toRadians(0));
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        // Cu traj4 s-a dus in spatele inelelor pentru a le trage in intake
        intakeRings(1);
        setShooterPower(1, 0.04);
        drive.followTrajectory(traj5);
        drive.turn(Math.toRadians(2));
        // Cu traj5 merge in fata si ia inelele in intake (primele 3)
        sleep(700);
        liftRingHolder();
        sleep(400);
        flicker();
        sleep(100);
        flicker();
        sleep(100);
        flicker();
        returnRingHolder();
        sleep(100);
        setShooterPower(1, 0.038);
        drive.followTrajectory(traj6);
        drive.turn(Math.toRadians(2));
        sleep(400);
        liftRingHolder();
        sleep(450);
        flicker();
        sleep(300);
        flicker();
        sleep(300);
        returnRingHolder();
        setShooterPower(0, 0);
        intakeRings(0);
        drive.followTrajectory(traj7);
        placeWobbleGoal(300, 0.3);
        sleep(500);
        drive.followTrajectory(traj8);
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
            sleep(300);
        }
    }

    private void pickWobbleGoal(int motorPosition, double motorPower){
        double wobbleGrabberGrab = 0, wobbleGrabberUngrab = 0.3;
        if (opModeIsActive()) {
            map.wobbleServo.setPosition(wobbleGrabberGrab);
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
        map.loaderFrontServo.setPosition(0.0);
        map.loaderBackServo.setPosition(0.0);
        map.feederServo.setPosition(0.0);
        map.intakeServo.setPosition(0.0);
        map.shooterServo.setPosition(0.0);
        map.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
