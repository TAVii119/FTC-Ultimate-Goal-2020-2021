package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.spartronics4915.lib.T265Camera;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import java.util.List;

import static java.lang.Math.atan2;

public class DriveSubsystem extends SubsystemBase {
    private final SampleMecanumDrive drive;
    private final boolean fieldCentric;
    private int controlMode;
    // Define 2 states, driver control or alignment control

    private final Pose2d startPosition = new Pose2d(0.0, 0.0, new Rotation2d());
    // A target vector we want the bot to align with
    private Vector2d towerPosition = new Vector2d(83.0, -36.2);
    private Vector2d rightPsPosition = new Vector2d(75.0, -18.5);
    private Vector2d centerPsPosition = new Vector2d(75.0, -8.5);
    private Vector2d leftPsPosition = new Vector2d(75.0, 4.5);

    public DriveSubsystem(SampleMecanumDrive drive, boolean isFieldCentric, Telemetry telemetry) {
        tele = telemetry;
        this.drive = drive;
        fieldCentric = isFieldCentric;
    }

    public void initialize() {

    }

    public void execute() {

    }

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // The position of the robot at the start of the Tele-Operated period
//    private final Pose2d startPosition = new Pose2d(-0.5, -14.7);

    Telemetry tele;

    public double distanceToTowergoal, currentY, currentX, currentHeading;

    public void setMode(DcMotor.RunMode mode) {
        drive.setMode(mode);
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients coefficients) {
        drive.setPIDFCoefficients(mode, coefficients);
    }

    public void update() {
        drive.update();
    }

    public void drive(double leftY, double leftX, double rightX) {
        headingController.setInputBounds(-Math.PI, Math.PI);

        distanceToTowergoal = towerPosition.getX() + currentX;
        com.acmerobotics.roadrunner.geometry.Pose2d driveDirection =
                new com.acmerobotics.roadrunner.geometry.Pose2d(leftY, -leftX, -rightX);
        drive.setWeightedDrivePower(driveDirection);
        // Update the localizer
        drive.getLocalizer().update();
//        tele.addData("Distance to Tower Goal", towerPosition.getX() - poseEstimate.getX());
//        tele.addData("x", poseEstimate.getX());
//        tele.addData("y", poseEstimate.getY());
//        tele.addData("Current heading: ", poseEstimate.getHeading());

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

    public boolean isBusy() {
        return drive.isBusy();
    }

//    public List<Double> getWheelVelocities() {
//        return drive.getWheelVelocities();
//    }

    public void stop() {
        drive(0, 0, 0);
    }

    @Override
    public void periodic() {
        drive.update();

    }
}