package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.TimedAction;

import java.util.concurrent.TimeUnit;

import static java.lang.Math.atan2;
import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class LocalizationSubsystem extends SubsystemBase {

    private Telemetry telemetry;
    private static T265Camera slamra = null;

    private Pose2d startPosition = new Pose2d(0, 0, new Rotation2d());
    private Vector2d towerPosition = new Vector2d(83.0, -36.2);
    private Vector2d rightPsPosition = new Vector2d(75.0, -18.5);
    private Vector2d centerPsPosition = new Vector2d(75.0, -8.5);
    private Vector2d leftPsPosition = new Vector2d(75.0, 4.5);

    public double currentY, currentX, currentHeading;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public LocalizationSubsystem(T265Camera slm, Telemetry tele) {
        this.slamra = slm;
        this.telemetry = tele;
    }

    public void initialize() {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
        slamra.setPose(startPosition);
    }

    public void runLocalization() {
        final int robotRadius = 9; // inches

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;

        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();
        currentHeading = rotation.getDegrees();
        currentX = translation.getX();
        currentY = translation.getY();

        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);
        double turretAngle;
        double turretAngleOffset;
//
//        if (getTurretAngle() > 70) {
//            turretAngleOffset = 70;
//        } else {
//            turretAngleOffset = getTurretAngle();
//        }
//
//        turretAngle = turretAngleOffset - currentHeading + 70;
//
//        double turretServoPosition = 0.46 / 140 * turretAngle;

        // Aici vin verificari sa nu depaseasca servo-ul o anumita pozitie

        telemetry.addData("Robot angle: ", getTurretAngle());
        telemetry.addData("Turret angle:", getTurretAngle() - currentHeading);
//        telemetry.addData("Calculated turret angle:", turretAngle);
//        telemetry.addData("Turret servo position:", turretServoPosition);
        telemetry.addData("Current X:", currentX);
        telemetry.addData("Current Y:", currentY);
        telemetry.addData("Current Heading:", currentHeading);

        telemetry.update();
        // Unghiu la tureta creste in sensul acelor de ceasornic
        // Unghiu la robot creste invers acelor de ceasornic
    }

    public double getTurretAngle() {
        return Math.toDegrees(turretAngleFormula(towerPosition.getX(), towerPosition.getY(), currentX, currentY));
    }

    public double turretAngleFormula(double towerX, double towerY, double robotX, double robotY) {
        double dX = towerX - robotX;
        double dY = towerY - robotY;
        double ang = atan2(dY, dX);

        return ang;
    }

    public void stopLocalization() {
        slamra.stop();
    }
}
