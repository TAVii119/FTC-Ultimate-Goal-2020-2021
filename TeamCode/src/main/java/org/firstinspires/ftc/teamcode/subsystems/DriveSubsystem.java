package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import java.util.List;

import static java.lang.Math.atan2;

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
//    private final Pose2d startPosition = new Pose2d(-0.5, -14.7);
    private final Pose2d startPosition = new Pose2d(10.0, -25.0);
    // A target vector we want the bot to align with
    private Vector2d towerPosition = new Vector2d(83.0, -36.2);
    private Vector2d rightPsPosition = new Vector2d(75.0, -18.5);
    private Vector2d centerPsPosition = new Vector2d(75.0, -8.5);
    private Vector2d leftPsPosition = new Vector2d(75.0, 4.5);
    Telemetry tele;

    public double distanceToTowergoal, currentY, currentX;

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
        distanceToTowergoal = towerPosition.getX() + poseEstimate.getX();
        currentY = poseEstimate.getY();
        currentX = poseEstimate.getX();

        // Update the localizer
        drive.getLocalizer().update();
//        tele.addData("Distance to Tower Goal", towerPosition.getX() - poseEstimate.getX());
//        tele.addData("x", poseEstimate.getX());
//        tele.addData("y", poseEstimate.getY());
//        tele.addData("Current heading: ", poseEstimate.getHeading());
        tele.addData("Robot angle: ", getTurretAngle());
        tele.addData("Turret angle:", getTurretAngle() + Math.toDegrees(poseEstimate.getHeading()));
        // Unghiu la tureta creste in sensul acelor de ceasornic
        // Unghiu la robot creste invers acelor de ceasornic
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

    public double setTurretPosition() {
        // Key reprezinta distantele Y la care se afla robotul
        // 0.36 extremitate stanga, 0.23 mijloc, 0.0 dreapta
        LUT<Double, Double> positions = new LUT<Double, Double>()
        {{
            // Right of TowerGoal
            add(-53.0, 0.27);
            add(-51.0, 0.257);
            add(-48.0, 0.25);
            add(-47.0, 0.24);
            add(-45.0, 0.239);
            add(-42.0, 0.238);
            add(-41.0, 0.237);
            add(-39.0, 0.236);
            add(-38.0, 0.226);

            // Middle
            add(-36.0, 0.2256);
            // Middle

            // Left of TowerGoal
            add(-34.0, 0.22);
            add(-30.0, 0.217);
            add(-25.0, 0.215);
            add(-20.0, 0.212);
            add(-15.0, 0.18);
            add(-13.0, 0.17);
            add(-10.0, 0.168);
            add(-8.0, 0.165);
            add(-5.0, 0.16);
            add(-3.0, 0.15);
            add(-1.0, 0.147);
            add(4.0, 0.132);
            add(7.0, 0.127);
            add(10.0, 0.12);
            add(15.0, 0.115);
        }};
        double position = positions.getClosest(currentY);

        return position;
    }

    public double getTurretAngle() {
        return Math.toDegrees(turretAngleFormula(towerPosition.getX(), towerPosition.getY(), currentX, currentY));
    }

    // 1 - towergoal
    public double turretAngleFormula(double towerX, double towerY, double robotX, double robotY) {
        double dX = towerX - robotX;
        double dY = towerY - robotY;
        double ang = atan2(dY, dX);

        return ang;
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