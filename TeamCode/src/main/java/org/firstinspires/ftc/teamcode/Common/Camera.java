package org.firstinspires.ftc.teamcode.Common;


import static org.firstinspires.ftc.teamcode.Common.StaticVariables.hardwareMap;
import static org.firstinspires.ftc.teamcode.Common.StaticVariables.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import org.firstinspires.ftc.teamcode.core.Hardware;

import java.util.List;
@Config
public class Camera {
    private Limelight3A limelight;
    private boolean activated;
    public static double cameraAngle = 44, cameraHeight = 450, cameraHeightOffset = 18;

    private LLResult result;
    private List<LLResultTypes.ColorResult> results;
    private LLResultTypes.ColorResult sample;
    private List<List<Double>> corners;

    public double sampleX = 0, sampleY = 0, sampleH = 0;

    public static double turretMultiplier = 0.0032;
    public static double turretInit = 0.05;
    public static double clawInit = 0.31;
    public int extendoPos = 0;
    public double turretPos = 0;
    public double clawPos = 0;
    public Camera() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        activated = false;

        extendoHome = new HomeMade();
        clawHome = new HomeMade();

        extendoHome.add(0, 0);
        extendoHome.add(2, 33);
        extendoHome.add(5, 44);
        extendoHome.add(10, 60);
        extendoHome.add(15, 68);
        extendoHome.add(20, 83);
        extendoHome.add(25, 98);
        extendoHome.add(30, 111);
        extendoHome.add(35, 123);
        extendoHome.add(40, 146);
        extendoHome.add(45, 167);
        extendoHome.add(50, 193);
        extendoHome.add(54, 230);

    }

    HomeMade extendoHome;
    HomeMade clawHome;

    public void update() {


        if (!activated) return;

        telemetry.addData("sampleX", sampleX);
        telemetry.addData("sampleY", sampleY);
        telemetry.addData("sampleH", sampleH);
        result = limelight.getLatestResult();
        telemetry.addData("state", result != null);

        if (result != null) {
            results = result.getColorResults();

            if (!results.isEmpty()) {
                for (int i = 0; i < results.size(); i++) {
                    sample = results.get(i);


                    telemetry.addData("tx", sample.getTargetXDegrees());
                    telemetry.addData("ty", sample.getTargetYDegrees());
//                    if (sampleX < -25 || sampleX > 25 || sampleY > 87 || sampleY < 40) {
//                        sampleX = 0; sampleY = 0;
//                        continue;
//                    }

                    //getCoordinates(sample.getTargetXDegrees(), sample.getTargetYDegrees());

                    corners = sample.getTargetCorners();

                    if (corners.size() == 4) {
                        double largeEdge = getLength(corners.get(0).get(0), corners.get(0).get(1), corners.get(1).get(0), corners.get(1).get(1));
                        double smallEdge = getLength(corners.get(1).get(0), corners.get(1).get(1), corners.get(2).get(0), corners.get(2).get(1));

                        if (largeEdge > smallEdge) {
                            sampleH = Math.atan2(corners.get(0).get(1) - corners.get(1).get(1), corners.get(0).get(0) - corners.get(1).get(0));
                        }
                        else {
                            sampleH = Math.atan2(corners.get(1).get(1) - corners.get(2).get(1), corners.get(1).get(0) - corners.get(2).get(0));
                        }

                        sampleH = Math.toDegrees(-sampleH + Math.PI);

                        if (sampleH > 180) sampleH = sampleH - 360;
                        if (sampleH < -180) sampleH = sampleH + 360;

                        if (sampleH < 0) sampleH += 180;
                    }
                    else continue;
                    getCoordinates(sample.getTargetXDegrees(), sample.getTargetYDegrees());

                    break;
                }

            }
        }

    }

    private double getLength(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2- x1, 2) + Math.pow(y2 - y1, 2));
    }

    public void start() {
        activated = true;
        limelight.start();
        sampleX = 0; sampleY = 0; sampleH = 0;
    }
    public void stop() {
        activated = false;
    }
    public void setHeight(double ticks) {
        cameraHeight = 0.103425 * ticks + cameraHeightOffset;
    }

    public static int biela = 50;

    public static int clawMultiplier = 9;

    public static double cameraOffset = 34;

    public static double test = 30;
    public static double testIntake = 30;

    public static int clawDivide = 720;
    public void getCoordinates(double tx, double ty) {
        telemetry.addData("lift ticks", cameraHeight);
        sampleY = cameraHeight * Math.tan(Math.toRadians(ty + cameraAngle)) - cameraOffset;
        sampleX = Math.sqrt(sampleY * sampleY + cameraHeight * cameraHeight) * Math.tan(Math.toRadians(tx));

        if ( sampleY != 0 && sampleX != 0) {

            double turretArmDifference = Math.sqrt(18 * 18 - sampleX * sampleX);
            sampleY -= turretArmDifference;
            extendoPos = (int)extendoHome.search(sampleY);

            //extendoPos -= (180 - extendoPos) / 35;

            double turretAngle = Math.toDegrees(Math.acos(sampleX / 18)) - 90;
            turretPos = turretInit + turretAngle  * turretMultiplier ;
            telemetry.addData("turret angle", turretAngle);
            clawPos = clawInit - ( sampleH - 90 - turretAngle) / clawDivide;

        }

        telemetry.addData("angle claw", clawPos);
        telemetry.addData("turretpos", turretPos);
        telemetry.addData("epos", extendoPos);
    }
    public double getDetectedX() {
        return sampleX;
    }
    public double getDetectedY() {
        return sampleY;
    }
    public double getDetectedH() {
        return sampleH;
    }
}
