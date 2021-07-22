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

    // Red Alliance
    private Vector2d towerPosition;
    private Vector2d rightPsPosition;
    private Vector2d centerPsPosition;
    private Vector2d leftPsPosition;

    private Vector2d startPose;

    public double currentY, currentX, currentHeading, xOffset = -8.54, yOffset = 0.0;
    public int alliance; // 0 - RED, 1 - BLUE

    public double turretPosition = 0, previousTurretPosition = 0;

    // -1 - Manual, 0 - Towergoal, 1 - Left PS, 2 - Center PS, 3 - Right PS
    public int currentTarget = 0;
    public double manualTurretServoPos;

    public int getCurrentTarget() { return currentTarget; }
    public void setCurrentTarget(int target) { currentTarget = target; }

    public void setManualTurretServoPos(double pos) { manualTurretServoPos = pos; }

    private static T265Camera slamra = null;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private double startX, startY;

    public LocalizationSubsystem(T265Camera slm, Telemetry tele, double startX, double startY, int alliance) {
        this.telemetry = tele;
        this.slamra = slm;
        this.startX = startX;
        this.startY = startY;

        this.alliance = alliance; // 0 - RED, 1 - BLUE
        this.startPose = new Vector2d(startX, startY);

        if (alliance == 0) {
            this.towerPosition = new Vector2d(83.0, -37.5);
            this.rightPsPosition = new Vector2d(77.0, -18.6);
            this.centerPsPosition = new Vector2d(77.0, -3.0);
            this.leftPsPosition = new Vector2d(77.0, 6.0);
        } else {
            this.towerPosition = new Vector2d(83.0, 36.0);
            this.rightPsPosition = new Vector2d(77.0, 10.5);
            this.centerPsPosition = new Vector2d(77.0, 25.5);
            this.leftPsPosition = new Vector2d(77.0, 36.5);
        }
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
        currentX = translation.getX() + xOffset + this.startPose.getX();
        currentY = translation.getY() + yOffset + this.startPose.getY();

        double turretAngle;
        double turretAngleOffset;

        Vector2d target = new Vector2d();

        if (currentTarget == 0)
            target = this.towerPosition;
        if (currentTarget == 1)
            target = this.leftPsPosition;
        if (currentTarget == 2)
            target = this.centerPsPosition;
        if (currentTarget == 3)
            target = this.rightPsPosition;

        if (getTurretAngle(target.getX(), target.getY()) > 72.5) {
            turretAngleOffset = 72.5;
        } else {
            turretAngleOffset = getTurretAngle(target.getX(), target.getY());
        }

        turretAngle = turretAngleOffset - currentHeading + 72.5;

        double turretServoPosition = 0.48 / 145 * turretAngle;

        double positionX = translation.getX() + startPose.getX(),
                positionY = translation.getY() + startPose.getY();

//        if (currentTarget == 0) {
//            turretServoPosition += 0.01;
//        }

        if (this.alliance == 0) {
            if (currentTarget == 0 && positionY < -32)
                turretServoPosition += 0.03;

            if (currentTarget == 0 && positionY < -45)
                turretServoPosition += 0.025;

            if (currentTarget == 0 && positionY > -25)
                turretServoPosition -= 0.02;

            if (currentTarget == 0 && positionY > -18)
                turretServoPosition -= 0.025;

            if (currentTarget == 0 && positionY > -11)
                turretServoPosition -= 0.025;

//            if (currentTarget == 0 && positionY >= -45
//                    && positionY <= -25
//            )
//                turretServoPosition -= 0.025;
        } else if (this.alliance == 1) {
            // BLUE
            if (currentTarget == 0 && positionY > 45)
                turretServoPosition -= 0.035;

            if (currentTarget == 0 && positionY < 38)
                turretServoPosition += 0.012;

            if (currentTarget == 0 && positionY < 33)
                turretServoPosition += 0.018;

            if (currentTarget == 0 && positionY < 21)
                turretServoPosition += 0.018;

            if (currentTarget == 0 && positionY < 15)
                turretServoPosition += 0.024;

            if (currentTarget == 0 && positionY < 12)
                turretServoPosition += 0.026;

            if (currentTarget == 0 && positionY < 8)
                turretServoPosition += 0.01;

            turretServoPosition += 0.01;
        }
        if (turretServoPosition > 0.48) {
            turretServoPosition = 0.48;
        }

        if (turretServoPosition < 0) {
            turretServoPosition = 0;
        }

        if (currentTarget != -1 && currentTarget != -2) {
            previousTurretPosition = turretServoPosition;
            turretPosition = turretServoPosition;
        }

        if (currentTarget == -1 || currentTarget == -2) {
            turretPosition = manualTurretServoPos;
        }

        telemetry.addData("Current target:", currentTarget);
        telemetry.addData("Robot angle:", getTurretAngle(towerPosition.getX(), towerPosition.getY()));
        telemetry.addData("Turret angle:", getTurretAngle(towerPosition.getX(), towerPosition.getY()) - currentHeading);
        telemetry.addData("Calculated turret angle:", turretAngle);
        telemetry.addData("Turret servo position:", turretServoPosition);
        telemetry.addData("Current X:", translation.getX() + startPose.getX());
        telemetry.addData("Current Y:", translation.getY() + startPose.getY());
        telemetry.addData("Current heading:", currentHeading);

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

        if (currentTarget == 0 || currentTarget == -1) {
            LUT<Double, Double> positions = new LUT<Double, Double>() {{
                add(0.0 + xOffset, 0.55);
                add(-12.2047 + xOffset, 0.54);
                add(-15.2047 + xOffset, 0.528);
                add(-40.9449 + xOffset, 0.51);
                add(-47.9449 + xOffset, 0.5);
            }};
            position = positions.getClosest(currentX);
        } else {
            LUT<Double, Double> positions = new LUT<Double, Double>() {{
                add(0.0 + xOffset, 0.52);
                add(-12.2047 + xOffset, 0.51);
                add(-40.9449 + xOffset, 0.5);
                add(-47.9449 + xOffset, 0.49);
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
