package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@Autonomous(name = "AutoRedRightFull", group = "Red Auto")
public class AutoRedRightFull extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    RingsDeterminationPipeline pipeline;
    SimpleHardware map = new SimpleHardware();

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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(30,175);

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
                .splineToLinearHeading(new Pose2d(-4.5, -59.5), 0)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-4.5, -59.5))
                .strafeTo(new Vector2d(-2, -36))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(-2, -36, Math.toRadians(180)))
                .lineTo(new Vector2d(-21, -19))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(-21, -19, Math.toRadians(181)))
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
        setShooterPower(1, 0.06);
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

    private void caseB(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-2, -50))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-2, -50, Math.toRadians(-3)))
                .strafeTo(new Vector2d(-2, -36))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(-2, -36))
                .lineTo(new Vector2d(16, -40))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(16, -40, Math.toRadians(180)))
                .lineTo(new Vector2d(-20.5, -19))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(-20.5, -19, Math.toRadians(179)))
                .lineTo(new Vector2d(-37, -17))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(-37, -17, Math.toRadians(-13)))
                .lineTo(new Vector2d(12, -50))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(new Pose2d(12.0, -50))
                .back(10)
                .build();

        resetServos();
        dropIntake();
        drive.followTrajectory(traj1);
        // Porneste shooter
        setShooterPower(1, 0.06);
        drive.followTrajectory(traj2);
        // Trage la tower goal
        liftRingHolder();
        sleep(500);
        flicker();
        flicker();
        flicker();
        flicker();
        setShooterPower(0, 0.06);
        returnRingHolder();
        drive.followTrajectory(traj3);
        // Lasa wobble goal
        placeWobbleGoal(450, 0.3);
        returnWobbleArm();
        sleep(500);
        drive.followTrajectory(traj4);
        placeWobbleGoal(500, 0.3);
        drive.followTrajectory(traj5);
        // Ia wobble goal
        pickWobbleGoal(100, 0.5);
        pickWobbleGoal(0, 0.3);
        sleep(300);
        drive.followTrajectory(traj6);
        placeWobbleGoal(450, 0.3);
        returnWobbleArm();
        sleep(250);
        drive.followTrajectory(traj7);
        // Parcheaza
    }

    private void caseC(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-2, -50))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-2, -50, Math.toRadians(-3)))
                .strafeTo(new Vector2d(-2, -36))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(-2, -36, Math.toRadians(-3)))
                .lineTo(new Vector2d(45, -59))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(45, -59, Math.toRadians(180)))
                .lineTo(new Vector2d(-20.5, -19))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(-20.5, -19, Math.toRadians(182))) //177
                .lineTo(new Vector2d(-35.5, -29))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(-35.5, -29, Math.toRadians(-48)))
                .lineTo(new Vector2d(32, -58))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(new Pose2d(32.0, -58))
                .back(30)
                .build();

        resetServos();
        dropIntake();
        drive.followTrajectory(traj1);
        // Porneste shooter
        setShooterPower(1, 0.06);
        drive.followTrajectory(traj2);
        // Trage la tower goal
        liftRingHolder();
        sleep(500);
        flicker();
        flicker();
        flicker();
        flicker();
        setShooterPower(0, 0.06);
        returnRingHolder();
        drive.followTrajectory(traj3);
        // Lasa wobble goal
        placeWobbleGoal(450, 0.3);
        returnWobbleArm();
        sleep(500);
        drive.followTrajectory(traj4);
        placeWobbleGoal(500, 0.3);
        drive.followTrajectory(traj5);
        // Ia wobble goal
        pickWobbleGoal(100, 0.5);
        pickWobbleGoal(0, 0.3);
        sleep(300);
        drive.followTrajectory(traj6);
        placeWobbleGoal(450, 0.3);
        returnWobbleArm();
        sleep(250);
        drive.followTrajectory(traj7);
        // Parcheaza
    }

        private void intakeRings(double power) {
        map.intakeMotor.setPower(power);
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
