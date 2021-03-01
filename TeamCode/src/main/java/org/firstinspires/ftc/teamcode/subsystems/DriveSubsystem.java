package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import java.util.List;

public class DriveSubsystem extends SubsystemBase {

    private final SampleMecanumDrive drive;
    private final boolean fieldCentric;
    private int controlMode;
    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_TOWER,
        ALIGN_TO_RIGHT_PS,
        ALIGN_TO_CENTER_PS,
        ALIGN_TO_LEFT_PS
    }

    private Mode currentMode = Mode.NORMAL_CONTROL;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // The position of the robot at the start of the Tele-Operated period
    private final Pose2d startPosition = new Pose2d(-63.0, -48.5);
    // A target vector we want the bot to align with
    private Vector2d towerPosition = new Vector2d(83.0, -36.2);
    private Vector2d rightPsPosition = new Vector2d(75.0, -19.0 + 5.512);
    private Vector2d centerPsPosition = new Vector2d(75.0, -11.0 + 5.512);
    private Vector2d leftPsPosition = new Vector2d(75.0, -2.0 + 5.512);
    Telemetry tele;

    public double distanceToTowergoal;

    public DriveSubsystem(SampleMecanumDrive drive, boolean isFieldCentric, Telemetry telemetry) {
        tele = telemetry;
        this.drive = drive;
        fieldCentric = isFieldCentric;
        drive.getLocalizer().setPoseEstimate(startPosition);
    }

    public void normalMode() {
        controlMode = 0;
    }

    public void alignToTower() {
        controlMode = 1;
    }

    public void alignToRightPs() {
        controlMode = 2;
    }

    public void alignToCenterPs() {
        controlMode = 3;
    }

    public void alignToLeftPs() {
        controlMode = 4;
    }

    public void setMode(DcMotor.RunMode mode) {
        drive.setMode(mode);
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients coefficients) {
        drive.setPIDFCoefficients(mode, coefficients);
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public void update() {
        drive.update();
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }



    public void drive(double leftY, double leftX, double rightX) {
        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);
        // Read pose
        Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

        // Declare a drive direction
        // Pose representing desired x, y, and angular velocity
        Pose2d driveDirection = new Pose2d();

        switch (currentMode) {
            case NORMAL_CONTROL:
                // Switch into alignment mode if a is pressed
                if (controlMode == 1) {
                    currentMode = Mode.ALIGN_TO_TOWER;
                }

                if (controlMode == 2) {
                    currentMode = Mode.ALIGN_TO_RIGHT_PS;
                }

                if (controlMode == 3) {
                    currentMode = Mode.ALIGN_TO_CENTER_PS;
                }

                if (controlMode == 4) {
                    currentMode = Mode.ALIGN_TO_LEFT_PS;
                }


                // Standard teleop control
                // Convert gamepad input into desired pose velocity
                driveDirection = new Pose2d(leftY, -leftX, -rightX);
                break;

            case ALIGN_TO_LEFT_PS:
                if (controlMode == 0) {
                    currentMode = Mode.NORMAL_CONTROL;
                }

                Vector2d fieldFrameInput = new Vector2d(leftY, -leftX);
                Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                Vector2d difference = leftPsPosition.minus(poseEstimate.vec());
                double theta = difference.angle();

                double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                headingController.setTargetPosition(theta);

                double headingInput = (headingController.update(poseEstimate.getHeading())
                        * DriveConstants.kV + thetaFF)
                        * DriveConstants.TRACK_WIDTH;

                // Combine the field centric x/y velocity with our derived angular velocity
                driveDirection = new Pose2d(
                        robotFrameInput,
                        headingInput
                );
                break;

            case ALIGN_TO_CENTER_PS:
                if (controlMode == 0) {
                    currentMode = Mode.NORMAL_CONTROL;
                }

                fieldFrameInput = new Vector2d(leftY, -leftX);
                robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                difference = centerPsPosition.minus(poseEstimate.vec());
                theta = difference.angle();

                thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                headingController.setTargetPosition(theta);

                headingInput = (headingController.update(poseEstimate.getHeading())
                        * DriveConstants.kV + thetaFF)
                        * DriveConstants.TRACK_WIDTH;

                // Combine the field centric x/y velocity with our derived angular velocity
                driveDirection = new Pose2d(
                        robotFrameInput,
                        headingInput
                );
                break;

            case ALIGN_TO_RIGHT_PS:
                if (controlMode == 0) {
                    currentMode = Mode.NORMAL_CONTROL;
                }
                fieldFrameInput = new Vector2d(leftY, -leftX);
                robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                difference = rightPsPosition.minus(poseEstimate.vec());
                theta = difference.angle();

                thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                headingController.setTargetPosition(theta);

                headingInput = (headingController.update(poseEstimate.getHeading())
                        * DriveConstants.kV + thetaFF)
                        * DriveConstants.TRACK_WIDTH;

                // Combine the field centric x/y velocity with our derived angular velocity
                driveDirection = new Pose2d(
                        robotFrameInput,
                        headingInput
                );
                break;

            case ALIGN_TO_TOWER:
                // Switch back into normal driver control mode if b is pressed
                if (controlMode == 0) {
                    currentMode = Mode.NORMAL_CONTROL;
                }

                // Create a vector from the gamepad x/y inputs which is the field relative movement
                // Then, rotate that vector by the inverse of that heading for field centric control
                fieldFrameInput = new Vector2d(leftY, -leftX);
                robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                // Difference between the target vector and the bot's position
                difference = towerPosition.minus(poseEstimate.vec());
                // Obtain the target angle for feedback and derivative for feedforward
                theta = difference.angle();

                // Not technically omega because its power. This is the derivative of atan2
                thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                // Set the target heading for the heading controller to our desired angle
                headingController.setTargetPosition(theta);

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                headingInput = (headingController.update(poseEstimate.getHeading())
                        * DriveConstants.kV + thetaFF)
                        * DriveConstants.TRACK_WIDTH;

                // Combine the field centric x/y velocity with our derived angular velocity
                driveDirection = new Pose2d(
                        robotFrameInput,
                        headingInput
                );
                break;
        }

        drive.setWeightedDrivePower(driveDirection);

        // Update the heading controller with our current heading
        headingController.update(poseEstimate.getHeading());
        distanceToTowergoal = towerPosition.getX() - poseEstimate.getX();

        // Update he localizer
        drive.getLocalizer().update();

        tele.addData("Distance to Tower Goal", towerPosition.getX() - poseEstimate.getX());
        tele.addData("x", poseEstimate.getX());
        tele.addData("y", poseEstimate.getY());
        tele.addData("heading", poseEstimate.getHeading());
        tele.update();
    }

    public double setRampPosition() {
        // Key reprezinta distantele fata de towergoal
        LUT<Double, Double> positions = new LUT<Double, Double>()
        {{
            add(82.0, 0.0425);
            add(92.449, 0.033);
            add(101.764, 0.035);
            add(110.998, 0.0354);
            add(120.447, 0.0348);
            add(130.211, 0.0245);
            add(139.463, 0.027);
        }};
        double position = positions.getClosest(distanceToTowergoal);

        return position;
    }

    public void setDrivePower(Pose2d drivePower) {
        drive.setDrivePower(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return drive.trajectoryBuilder(startPose, startHeading);
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void turn(double radians) {
        drive.turnAsync(radians);
    }

    public List<Double> getWheelVelocities() {
        return drive.getWheelVelocities();
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public Pose2d getPoseVelocity() {
        return drive.getPoseVelocity();
    }

    public Localizer getLocalizer() {
        return drive.getLocalizer();
    }

    @Override
    public void periodic() {
        drive.update();
    }
}