package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import java.util.Vector;

import static java.lang.Math.atan2;

public class LocalizationSubsystem extends SubsystemBase {

    private Telemetry telemetry;
    private Vector2d towerPosition = new Vector2d(83.0, -37.5);
    private Vector2d rightPsPosition = new Vector2d(77.0, -46.6);
    private Vector2d centerPsPosition = new Vector2d(77.0, -37.0);
    private Vector2d leftPsPosition = new Vector2d(77.0, -25.0);

    private Vector2d startPose = new Vector2d(-63.0, -24.3);

    public double currentY, currentX, currentHeading, xOffset = -8.54, yOffset = 0.0;

    public double turretPosition = 0, previousTurretPosition = 0;

    // -1 - Manual, 0 - Towergoal, 1 - Left PS, 2 - Center PS, 3 - Right PS
    public int currentTarget = 0;
    public double manualTurretServoPos;

    public int getCurrentTarget() { return currentTarget; }
    public void setCurrentTarget(int target) { currentTarget = target; }

    public void setManualTurretServoPos(double pos) { manualTurretServoPos = pos; }

    private static T265Camera slamra = null;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public LocalizationSubsystem(T265Camera slm, Telemetry tele) {
        this.telemetry = tele;
        this.slamra = slm;
    }

    boolean isPoseSet = false;

    public void initialize() {
        slamra.stop();
        slamra.start();

//        if (!isPoseSet) {
//            if (slamra.getLastReceivedCameraUpdate().confidence == T265Camera.PoseConfidence.High) {
//                slamra.setPose(startPose);
//            }
//        }
    }

    public void loop() {
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

        currentHeading = rotation.getDegrees();
        currentX = translation.getX() + xOffset + startPose.getX();
        currentY = translation.getY() + yOffset + startPose.getY();

        double turretAngle;
        double turretAngleOffset;

        Vector2d target = new Vector2d();

        if (currentTarget == 0)
            target = towerPosition;
        if (currentTarget == 1)
            target = leftPsPosition;
        if (currentTarget == 2)
            target = centerPsPosition;
        if (currentTarget == 3)
            target = rightPsPosition;

        if (getTurretAngle(target.getX(), target.getY()) > 72.5) {
            turretAngleOffset = 72.5;
        } else {
            turretAngleOffset = getTurretAngle(target.getX(), target.getY());
        }

        turretAngle = turretAngleOffset - currentHeading + 72.5;

        double turretServoPosition = 0.42 / 145 * turretAngle;

        if (currentTarget == 0) {
            turretServoPosition += 0.01;
        }

        if (currentTarget == 0 && translation.getY() + startPose.getY() <= -40) {
            turretServoPosition -= 0.02;
        }

        if (currentTarget > 0)
            turretServoPosition += 0.02;

        if (turretServoPosition > 0.42) {
            turretServoPosition = 0.42;
        }

        if (turretServoPosition < 0) {
            turretServoPosition = 0;
        }

        if (currentTarget == 0 && translation.getY() + startPose.getY() >= -20
        && currentTarget == 0 && translation.getY() + startPose.getX() <= -10)
            turretServoPosition -= 0.043;

        if (currentTarget == 0 && translation.getY() + startPose.getY() >= -10)
            turretServoPosition -= 0.02;

        if (currentTarget == 0 && translation.getY() + startPose.getY() < -20)
            turretServoPosition -= 0.035;

        if (currentTarget == 0 && translation.getY() + startPose.getY() < -30)
            turretServoPosition += 0.025;

        if (currentTarget != -1) {
            previousTurretPosition = turretServoPosition;
            turretPosition = turretServoPosition;
        }

        if (currentTarget == -1) {
            turretPosition = manualTurretServoPos;
        }

        telemetry.addData("Robot angle: ", getTurretAngle(towerPosition.getX(), towerPosition.getY()));
        telemetry.addData("Turret angle:", getTurretAngle(towerPosition.getX(), towerPosition.getY()) - currentHeading);
        telemetry.addData("Calculated turret angle:", turretAngle);
        telemetry.addData("Turret servo position:", turretServoPosition);
        telemetry.addData("Current X:", translation.getX() + startPose.getX());
        telemetry.addData("Current Y:", translation.getY() + startPose.getY());
        telemetry.addData("Current Heading:", currentHeading);

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
        // Unghiu la tureta creste in sensul acelor de ceasornic
        // Unghiu la robot creste invers acelor de ceasornic
    }

    public double getTurretPosition() { return turretPosition; }

    public double getTurretAngle(double targetX, double targetY) {
        return Math.toDegrees(targetAngleFormula(targetX, targetY, currentX, currentY));
    }

    public double getRampPosition() {
        double position;

        if (currentTarget == 0) {
            LUT<Double, Double> positions = new LUT<Double, Double>() {{
                add(0.0 + xOffset, 0.29);
                add(-12.2047 + xOffset, 0.28);
                add(-40.9449 + xOffset, 0.27);
            }};
            position = positions.getClosest(currentX);
        } else {
            LUT<Double, Double> positions = new LUT<Double, Double>() {{
                add(0.0 + xOffset, 0.27);
                add(-6.0 + xOffset, 0.275);
                add(-12.2047 + xOffset, 0.28);
                add(-40.9449 + xOffset, 0.285);
            }};
            position = positions.getClosest(currentX);
        }

        return position;
    }

    public double targetAngleFormula(double targetX, double targetY, double robotX, double robotY) {
        double dX = targetX - robotX;
        double dY = targetY - robotY;
        double ang = atan2(dY, dX);

        return ang;
    }

    public void stopLocalization() {
        slamra.stop();
    }
}
