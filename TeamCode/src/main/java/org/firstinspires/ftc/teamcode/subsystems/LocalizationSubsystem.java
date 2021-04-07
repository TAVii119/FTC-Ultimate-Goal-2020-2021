package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private Vector2d towerPosition = new Vector2d(83.0, -36.2);
    private Vector2d rightPsPosition = new Vector2d(75.0, -18.5);
    private Vector2d centerPsPosition = new Vector2d(75.0, -8.5);
    private Vector2d leftPsPosition = new Vector2d(75.0, 4.5);

    public double currentY, currentX, currentHeading, xOffset = -8.54, yOffset = 3.74;

    public double turretPosition = 0;

    // 0 - Towergoal, 1 - Left PS, 2 - Center PS, 3 - Right PS
    public int currentTarget = 0;

    public int getCurrentTarget() { return currentTarget; }
    public void setCurrentTarget(int target) { currentTarget = target; }

    private static T265Camera slamra = null;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public LocalizationSubsystem(T265Camera slm, Telemetry tele) {
        this.telemetry = tele;
        this.slamra = slm;
    }

    public void initialize() {
        slamra.stop();
        slamra.start();
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
        currentX = translation.getX() + xOffset;
        currentY = translation.getY() + yOffset;

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

        if (getTurretAngle(target.getX(), target.getY()) > 70) {
            turretAngleOffset = 70;
        } else {
            turretAngleOffset = getTurretAngle(target.getX(), target.getY());
        }

        turretAngle = turretAngleOffset - currentHeading + 70;

        double turretServoPosition = 0.46 / 140 * turretAngle;

        if (turretServoPosition > 0.46) {
            turretServoPosition = 0.46;
        }

        if (turretServoPosition < 0) {
            turretServoPosition = 0;
        }

        if (turretServoPosition < 0.2) {
            turretServoPosition -= 0.025;
        }
        turretPosition = turretServoPosition;
        // Aici vin verificari sa nu depaseasca servo-ul o anumita pozitie

        telemetry.addData("Robot angle: ", getTurretAngle(towerPosition.getX(), towerPosition.getY()));
        telemetry.addData("Turret angle:", getTurretAngle(towerPosition.getX(), towerPosition.getY()) - currentHeading);
        telemetry.addData("Calculated turret angle:", turretAngle);
        telemetry.addData("Turret servo position:", turretServoPosition);
        telemetry.addData("Current X:", translation.getX());
        telemetry.addData("Current Y:", translation.getY());
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
        LUT<Double, Double> positions = new LUT<Double, Double>()
        {{
            add(0.0 + xOffset, 0.12);
            add(-12.2047 + xOffset, 0.115);
            add(-40.9449 + xOffset, 0.13);
        }};
        double position = positions.getClosest(currentX);

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
