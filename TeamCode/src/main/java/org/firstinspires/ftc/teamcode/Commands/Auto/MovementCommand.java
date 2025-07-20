package org.firstinspires.ftc.teamcode.Commands.Auto;

import static org.firstinspires.ftc.teamcode.Common.StaticVariables.hardwareMap;
import static org.firstinspires.ftc.teamcode.Common.StaticVariables.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Common.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Common.PIDF;
import org.firstinspires.ftc.teamcode.core.Hardware;
import org.opencv.core.Mat;

import java.util.Collections;
import java.util.Set;

@Config
public class MovementCommand implements Command {

    public static boolean parked = false;
    public static double robotX, robotY, robotH;
    public static double deltaDistance = 0.d, deltaAngle = 0.d;

    private static PIDF distancePID = new PIDF(), turningPID = new PIDF();
    public static double distanceP = 0.022, distanceI = 0, distanceD = 0.003, distanceF = 0.15;
    public static double turningP = 0.022, turningI = 0.0, turningD = 0.002, turningF = 0;
    public static double correctionK = 0.1;
    public static double corK = 0.1;
    private static double targetX = 0, targetY = 0, targetH = 90, maximumSpeed, startX, startY;
    private static double pathAngle;
    public static double robotVelocityX, robotVelocityY, robotVelocityW;
    public static void setNewTargetPoint(double targetx, double targety, double maximumspeed) {
        targetX = targetx;
        targetY = targety;
        maximumSpeed = maximumspeed;

        startX = robotX;
        startY = robotY;

        pathAngle = Math.atan2(targetY - startY, targetX - startX);

        distancePID.setCoefficients(distanceP, distanceI, distanceD, distanceF);
        distancePID.setTargetSpeed(maximumSpeed);
        distancePID.resetReference();

        turningPID.setCoefficients(turningP, turningI, turningD, turningF);
        turningPID.setTargetSpeed(maximumSpeed);
        turningPID.resetReference();
    }

    public static void setNewTargetHeading(double targeth) {
        targetH = targeth;

        distancePID.setCoefficients(distanceP, distanceI, distanceD, distanceF);
        distancePID.setTargetSpeed(maximumSpeed);
        distancePID.resetReference();

        turningPID.setCoefficients(turningP, turningI, turningD, turningF);
        turningPID.setTargetSpeed(maximumSpeed);
        turningPID.resetReference();
    }

    public GoBildaPinpointDriver odometry;

    Hardware _hardware;

    public MovementCommand(Hardware hw)
    {
        _hardware = hw;
        odometry = _hardware.odometry;
    }

    @Override
    public void initialize() {

    }

    public boolean angleWalk = false;
    @Override
    public void execute() {

        if ( angleWalk ) {
            targetH = Math.toDegrees(pathAngle);
            if ( targetH < 0 )
                targetH += 180;
        }
        UpdateOdometry();
        if ( parked ) {
            robotVelocityX = 0;
            robotVelocityY = 0;
            robotVelocityW = 0;
            return;
        }

        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double deltaH = targetH - robotH;

        while (deltaH > 360) deltaH -= 360;
        while (deltaH < -360) deltaH += 360;

        if (deltaH > 180) deltaH -= 360;
        if (deltaH < -180) deltaH += 360;

        double distance = Math.sqrt((Math.pow(deltaX, 2) + Math.pow(deltaY, 2)));

        deltaDistance = distance;
        deltaAngle = deltaH;

        double absoluteAngle = Math.atan2(deltaY, deltaX);

        distancePID.setCoefficients(distanceP, distanceI, distanceD, distanceF);
        double speed = distancePID.getOutput(distance);

        turningPID.setCoefficients(turningP, turningI, turningD, turningF);
        double turningSpeed = turningPID.getOutput(deltaH);

        if ( angleWalk )
            correctionK = corK;
        else correctionK = 0.0;

        double correctionSpeed = getPerpendicular(startX, startY, robotX, robotY, targetX, targetY) * correctionK;
        double correctionForceAngle = pathAngle + getSign(startX, startY, robotX, robotY, targetX, targetY) * Math.PI / 2;

        double deltaAngle = absoluteAngle - correctionForceAngle;

        speed = limitSpeed(speed, maximumSpeed);
        turningSpeed = limitSpeed(turningSpeed, maximumSpeed);
        correctionSpeed = limitSpeed(correctionSpeed, maximumSpeed);

        double quadraticA = 1;
        double quadraticB = 2 * correctionSpeed * Math.cos(deltaAngle);
        double quadraticC = Math.pow(correctionSpeed, 2) - Math.pow(speed, 2);

        double scaledSpeed = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2) - 4 * quadraticA * quadraticC)) / 2 * quadraticA;

        if (Double.isNaN(scaledSpeed)) scaledSpeed = 0;

        if (speed < 0) scaledSpeed = -scaledSpeed;

        if (distance < 1) distancePID.disableIF();
        if (distance > 2) distancePID.enableIF();

        if (Math.abs(deltaH) < 1) turningPID.disableIF();
        if (Math.abs(deltaH) > 2) turningPID.enableIF();

        double cosTrick = 0;

        if (Math.abs(deltaH) > 90) cosTrick = 0;
        else cosTrick = Math.cos(Math.toRadians(deltaH));

        robotVelocityX = (Math.cos(absoluteAngle) * scaledSpeed + Math.cos(correctionForceAngle) * correctionSpeed) * cosTrick;
        robotVelocityY = (Math.sin(absoluteAngle) * scaledSpeed + Math.sin(correctionForceAngle) * correctionSpeed) * cosTrick;
        robotVelocityW = -turningSpeed;
        //robotVelocityW = 0;

        telemetry.addData("deltaH", deltaH);

    }

    @Override
    public void end(boolean interrupted) {
        Command.super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void UpdateOdometry()
    {
        odometry.update();
        Pose2D pos;

        pos = odometry.getPosition();

        if (!Double.isNaN(pos.getY(DistanceUnit.CM))) robotX = - pos.getY(DistanceUnit.CM);
        if (!Double.isNaN(pos.getX(DistanceUnit.CM))) robotY = pos.getX(DistanceUnit.CM);
        if (!Double.isNaN(pos.getHeading(AngleUnit.DEGREES))) robotH = pos.getHeading(AngleUnit.DEGREES) + 90;

        telemetry.addData("robotX", robotX);
        telemetry.addData("robotY", robotY);
        telemetry.addData("robotH", robotH);
    }

    private double limitSpeed(double speed, double maximum) {
        if (speed > maximum) speed = maximum;
        if (speed < -maximum) speed = -maximum;

        return speed;
    }
    private double getLength(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2- x1, 2) + Math.pow(y2 - y1, 2));
    }

    public double getPathAngle()
    {
        return pathAngle;
    }
    private double getPerpendicular(double x1, double y1, double x2, double y2, double x3, double y3) {
        double l1 = getLength(x1, y1, x2, y2);
        double l2 = getLength(x1, y1, x3, y3);
        double l3 = getLength(x2, y2, x3, y3);

        double p = (l1 + l2 + l3) / 2;

        double area = Math.sqrt(p * (p - l1) * (p - l2) * (p - l3));

        if (l2 == 0)
            return 0;

        return 2 * area / l2;
    }

    private double getSign(double x1, double y1, double x2, double y2, double x3, double y3) {
        double det = x1 * y2 + x2 * y3 + x3 * y1 - x3 * y2 - y3 * x1 - x2 * y1;

        if (det > 0) return 1;
        else return -1;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }
}
