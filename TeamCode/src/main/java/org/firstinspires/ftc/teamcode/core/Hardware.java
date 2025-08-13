package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.Common.StaticVariables.hardwareMap;
import static org.firstinspires.ftc.teamcode.Common.StaticVariables.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commands.Auto.ScanSubmerssibleRepeat;
import org.firstinspires.ftc.teamcode.Common.Camera;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Common.StaticVariables;
import org.firstinspires.ftc.teamcode.core.Intake.Claw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialClaw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Intake.Intake;
import org.firstinspires.ftc.teamcode.core.Intake.V4b;

@Config
public class Hardware implements Mechanism {

    public GoBildaPinpointDriver odometry;

    public Chasis _chasis;
    public OuttakeDifferential differential;
    public DifferentialClaw differentialClaw;
    public DifferentialIntake differentialIntake;
    public Controller1 _controller1;
    public Controller2 _controller2;

    public Extendo _extendo;

    //public V4b v4b;

    //public Claw _claw;

    public Intake _intake;

    public Lift _lift;
    public DigitalChannel transferDetection;

    public ClawOutake _clawOutake;

    public Hooks _hooks;
    public Pto _pto;

    public Camera camera;

    public DigitalChannel distSensor;
    public static double offsetClaw = 0.5;
    public static double offsetIntake = 0.3;
    public static boolean lastSensorIntakeState = true;
    VoltageSensor baterie;
    public Hardware(HardwareMap hardwareMap, Gamepad gm1, Gamepad gm2, Telemetry tel)
    {
        StaticVariables.init(hardwareMap, gm1, gm2, tel);
        _chasis = new Chasis(this);
        _controller1 = new Controller1(this);
        _controller2 = new Controller2(this);
        differential = new OuttakeDifferential(this, "servoDifferentialLeft", "servoDifferentialRight", 0.3);
        differentialIntake = new DifferentialIntake(this, "servoDifferentialIntakeLeft", "servoDifferentialIntakeRight", offsetIntake);
        differentialClaw = new DifferentialClaw(this, "servoDifferentialClawRight", "servoDifferentialClawLeft", offsetClaw);
        _extendo = new Extendo(this);
        //v4b = new V4b(this);
        //_claw = new Claw(this);
        _intake = new Intake();
        _lift = new Lift();
        transferDetection = hardwareMap.get(DigitalChannel.class, "transferDetection");
        _clawOutake = new ClawOutake();
        _hooks = new Hooks();
        _pto = new Pto();


        camera = new Camera();

        distSensor = hardwareMap.get(DigitalChannel.class, "intakeSensor");

        baterie = hardwareMap.voltageSensor.iterator().next();
    }

    public double getVoltage()
    {
        return baterie.getVoltage();
    }

    @Override
    public void initialize() {
        _chasis.initialize();
        differential.SetState(OuttakeDifferential.DifferentialStates.START);
        differentialClaw.Initialize();
        differentialIntake.Initialize();

        _extendo.SetState(Extendo.ExtendoStates.INIT);
        //v4b.initialize();
        //_claw.Initialize();
        _intake.Initialize();
        _lift.Initialize();
        transferDetection.setMode(DigitalChannel.Mode.INPUT);
        distSensor.setMode(DigitalChannel.Mode.INPUT);
        _clawOutake.Initialize();
        _hooks.Initialize();
        _pto.Initialize();


        _controller1.initialize();
        _controller2.initialize();

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        odometry.setOffsets(49.78,-86.59, DistanceUnit.MM);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odometry.resetPosAndIMU();
        odometry.recalibrateIMU();

        camera.start();
    }


    boolean justStarted = true;
    @Override
    public void update() {
        if ( justStarted ) {
            differential.SetState((OuttakeDifferential.DifferentialStates.INTERMEDIATE));
            _hooks.SetState(Hooks.HooksState.INTERMEDIATE);
            justStarted = false;
        }
        telemetry.addData("odox", odometry.getEncoderX());
        telemetry.addData("odoy", odometry.getEncoderY());
        _extendo.update();
        _lift.Update();

        if ( StaticVariables.teleOp) {
            _controller1.update();
            _controller2.update();
        }
        lastSensorIntakeState = distSensor.getState();

        telemetry.addData("last intake", lastSensorIntakeState);

        camera.setHeight(_lift.getCurrentPosition());
        camera.update();
        telemetry.addData("tap", transferDetection.getState());
        //telemetry.addData("Claw rotation", _claw.getClawRotation());
        telemetry.update();
    }

    @Override
    public void finalize() {

    }
}
