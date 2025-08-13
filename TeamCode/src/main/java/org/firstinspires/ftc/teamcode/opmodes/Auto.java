package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Common.StaticVariables.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Commands.Auto.ChangePosition;
import org.firstinspires.ftc.teamcode.Commands.Auto.Extend;
import org.firstinspires.ftc.teamcode.Commands.Auto.PickupAfter;
import org.firstinspires.ftc.teamcode.Commands.Auto.RetractExtendo;
import org.firstinspires.ftc.teamcode.Commands.Auto.Scan;
import org.firstinspires.ftc.teamcode.Commands.Auto.ScanSubmerssibleRepeat;
import org.firstinspires.ftc.teamcode.Commands.Auto.ScoreHigh;
import org.firstinspires.ftc.teamcode.Commands.Auto.ScoreHighSubmersible;
import org.firstinspires.ftc.teamcode.Commands.Auto.WaitForAngle;
import org.firstinspires.ftc.teamcode.Commands.Auto.WaitForExtendo;
import org.firstinspires.ftc.teamcode.Commands.Auto.WaitForLift;
import org.firstinspires.ftc.teamcode.Commands.IntakeExtend;
import org.firstinspires.ftc.teamcode.Commands.IntakeRetract;
import org.firstinspires.ftc.teamcode.Commands.Auto.MovementCommand;
import org.firstinspires.ftc.teamcode.Commands.Auto.WaitForPosition;
import org.firstinspires.ftc.teamcode.Commands.Pickup;
import org.firstinspires.ftc.teamcode.Commands.Auto.PickupAuto;
import org.firstinspires.ftc.teamcode.Common.StaticVariables;
import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Hooks;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialClaw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Lift;
import org.firstinspires.ftc.teamcode.core.OuttakeDifferential;

@Config
@Autonomous
public class Auto extends CommandOpMode {
    SequentialCommandGroup testCommand;
    ParallelRaceGroup finalCommand;

    Hardware _hardware;
    MovementCommand movement;
    public static double xScore = 36, yScore = -28, hScore = 65;


    public static double xRight = 51, yRight = -30, hRight = 64, intakeRotRight = 0.13; // sample right
    public static double xCenter = 56, yCenter = -35, hCenter = 90, intakeRotCenter = -0.04d; // sample center
    public static double xLeft = 56, yLeft = -35, hLeft = 90, intakeRotLeft = 0.32d; // sample left
    public static double xSub = -42, ySub = -155, hSub = 20; // sample left
    public static double xMid = -42, yMid = -155, hMid =20; // sample left

    public static double rightClaw = 0.29;
    public static double centerClaw = 0.26;
    public static double leftClaw = 0.4;

    public static double rightDiff = 0.295;
    public static double centerDiff = 0.13;
    public static double leftDiff = 0.13;

    public static int rightExtendo = 138;
    public static int centreExtendo = 98;
    public static int leftExtendo = 143;
    public static int delayScan = 30;

    @Override
    public void initialize() {



        StaticVariables.teleOp = false;
        CommandScheduler.getInstance().reset();
        _hardware = new Hardware(hardwareMap, gamepad1, gamepad2, telemetry);
        _hardware.initialize();

        movement = new MovementCommand(_hardware);

        MovementCommand.parked = false;
        CommandScheduler.getInstance().schedule(movement);

        testCommand = new SequentialCommandGroup(
                new ChangePosition(xRight, yRight, hRight),
                new Extend(_hardware, rightExtendo),
                new InstantCommand(()->{
                    _hardware.differentialIntake.setPosition(DifferentialIntake.LINEAR_ANGLE_SCAN - 0.02, intakeRotRight);
                    _hardware.differentialClaw.setPosition(DifferentialClaw.LINEAR_ANGLE_SCAN, rightClaw);
                }),
                new ScoreHigh(_hardware, xRight, yRight, hRight, rightDiff, 1100),

                new ChangePosition(xCenter, yCenter, hCenter),
                new InstantCommand(()->{_hardware.differentialIntake.SetState(DifferentialIntake.IntakeDifferentialStates.TRANSFER);}),
                new WaitCommand(230),

                new RetractExtendo(_hardware, 900, intakeRotCenter, centerClaw),
                new Extend(_hardware, centreExtendo),
                new InstantCommand(()->{_hardware.differentialIntake.setPosition(DifferentialIntake.LINEAR_ANGLE_SCAN - 0.02, intakeRotCenter);
//                _hardware.differentialClaw.setPosition(DifferentialClaw.LINEAR_ANGLE_SCAN, centerClaw);
               }),
                new ScoreHigh(_hardware, xCenter, yCenter, hCenter, rightDiff, 300),

                new ChangePosition(xCenter, yCenter, hCenter ),
                new RetractExtendo(_hardware, 900, intakeRotLeft - 0.12, leftClaw),
                new InstantCommand(()->{_hardware.differentialIntake.setPosition(0.37, intakeRotLeft - 0.12);}),
                new Extend(_hardware, leftExtendo),
                new WaitForExtendo(15),
                new InstantCommand(()->{_hardware.differentialIntake.setPosition(0.38, intakeRotLeft);
//                    _hardware.differentialClaw.setPosition(DifferentialClaw.LINEAR_ANGLE_SCAN, leftClaw);
                }),
                new WaitCommand(10),
                new ScoreHigh(_hardware, xLeft, yLeft, hLeft, centerDiff, 550),

                new ChangePosition(xCenter, yCenter, hCenter),
                new WaitCommand(300),
                new RetractExtendo(_hardware),
                new ScoreHighSubmersible(_hardware, xMid, yMid, hMid, movement, 50),

                new InstantCommand(()-> {movement.angleWalk = true;}),

                new WaitForPosition(60),
                new InstantCommand(()-> {
                    movement.angleWalk = false;
                    _hardware._lift.SetState(Lift.LiftStates.CAMERA);
                }),
                new ChangePosition(xSub, ySub, hSub),
                //new WaitForPosition(15),
                new ParallelCommandGroup(
                    new WaitUntilCommand(()-> _hardware.odometry.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES) < 1),
                    new WaitUntilCommand(()-> _hardware.odometry.getVelX(DistanceUnit.MM) < 15),
                    new WaitUntilCommand(()-> _hardware.odometry.getVelY(DistanceUnit.MM) < 15)
                ),
                //new WaitCommand(200),
                new ScanSubmerssibleRepeat(_hardware),
                new ParallelCommandGroup(
                new RetractExtendo(_hardware, 600),
                new InstantCommand(()->{movement.pidClose();}),
                new ChangePosition(xScore, yScore, hScore),
                new InstantCommand(()-> {movement.angleWalk = true;
                    movement.pidBasket();

                })),
                new WaitForPosition(120),
                new InstantCommand(()-> {
                    movement.angleWalk = false;
                    _hardware._extendo.SetState(Extendo.ExtendoStates.SCORING);
                    _hardware._lift.SetState(Lift.LiftStates.HIGH_BASKET_AUTO);
                }),
                new ScoreHighSubmersible(_hardware, xMid, yMid, hMid, movement),

                new InstantCommand(()-> {movement.angleWalk = true;}),
                new WaitForPosition(60),
                new InstantCommand(()-> {
                    movement.angleWalk = false;
                    _hardware._lift.SetState(Lift.LiftStates.CAMERA);
                }),
                new ChangePosition(xSub, ySub, hSub),
                new ParallelCommandGroup(
                        new WaitUntilCommand(()-> _hardware.odometry.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES) < 1),
                        new WaitUntilCommand(()-> _hardware.odometry.getVelX(DistanceUnit.MM) < 15),
                        new WaitUntilCommand(()-> _hardware.odometry.getVelY(DistanceUnit.MM) < 15)
                ),                //new WaitCommand(200),
                new ScanSubmerssibleRepeat(_hardware),
                new ParallelCommandGroup(
                new RetractExtendo(_hardware, 600),
                new InstantCommand(()->{movement.pidClose();}),
                new ChangePosition(xScore, yScore, hScore),
                new InstantCommand(()-> {movement.angleWalk = true;
                    movement.pidBasket();
                })),
                new WaitForPosition(120),
                new InstantCommand(()-> {
                    movement.angleWalk = false;
                    _hardware._extendo.SetState(Extendo.ExtendoStates.SCORING);
                    _hardware._lift.SetState(Lift.LiftStates.HIGH_BASKET_AUTO);
                }),
                new ScoreHighSubmersible(_hardware, xMid, yMid, hMid, movement),

                new InstantCommand(()-> {movement.angleWalk = true;}),
                new WaitForPosition(60),
                new InstantCommand(()-> {
                    movement.angleWalk = false;
                    _hardware._lift.SetState(Lift.LiftStates.CAMERA);
                }),
                new ChangePosition(xSub, ySub, hSub),
                new ParallelCommandGroup(
                        new WaitUntilCommand(()-> _hardware.odometry.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES) < 1),
                        new WaitUntilCommand(()-> _hardware.odometry.getVelX(DistanceUnit.MM) < 15),
                        new WaitUntilCommand(()-> _hardware.odometry.getVelY(DistanceUnit.MM) < 15)
                ),                //new WaitCommand(200),
                new ScanSubmerssibleRepeat(_hardware),
                new ParallelCommandGroup(
                new RetractExtendo(_hardware, 600),
                new InstantCommand(()->{movement.pidClose();}),
                new ChangePosition(xScore, yScore, hScore),
                new InstantCommand(()-> {movement.angleWalk = true;
                    movement.pidBasket();
                })),
                new WaitForPosition(120),
                new InstantCommand(()-> {
                    movement.angleWalk = false;
                    _hardware._extendo.SetState(Extendo.ExtendoStates.SCORING);
                    _hardware._lift.SetState(Lift.LiftStates.HIGH_BASKET_AUTO);
                }),
                new ScoreHighSubmersible(_hardware, xMid, yMid, hMid, movement),

                new InstantCommand(()-> {movement.angleWalk = true;}),
                new WaitForPosition(60),
                new InstantCommand(()-> {
                    movement.angleWalk = false;
                    _hardware._lift.SetState(Lift.LiftStates.CAMERA);
                }),
                new ChangePosition(xSub, ySub , hSub),
                new ParallelCommandGroup(
                        new WaitUntilCommand(()-> _hardware.odometry.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES) < 1),
                        new WaitUntilCommand(()-> _hardware.odometry.getVelX(DistanceUnit.MM) < 15),
                        new WaitUntilCommand(()-> _hardware.odometry.getVelY(DistanceUnit.MM) < 15)
                ),                //new WaitCommand(200),
                new ScanSubmerssibleRepeat(_hardware),
                new ParallelCommandGroup(
                new RetractExtendo(_hardware, 600),
                new InstantCommand(()->{movement.pidClose();}),
                new ChangePosition(xScore, yScore, hScore),
                new InstantCommand(()-> {movement.angleWalk = true;
                    movement.pidBasket();
                })),
                new WaitForPosition(120),
                new InstantCommand(()-> {
                    movement.angleWalk = false;
                    _hardware._extendo.SetState(Extendo.ExtendoStates.SCORING);
                    _hardware._lift.SetState(Lift.LiftStates.HIGH_BASKET_AUTO);
                }),

                new ScoreHighSubmersible(_hardware, xSub + 15, -100, -90, movement),

                new InstantCommand(()-> {
                    movement.pidClose();
                    movement.angleWalk = false;
                    _hardware.differential.SetState(OuttakeDifferential.DifferentialStates.PARKING_AUTO);
                }),
                new ChangePosition(xSub + 15, -100, -90)
        );
        CommandScheduler.getInstance().schedule(
                testCommand
        );
        while ( opModeInInit())
        {
            _hardware.odometry.update();
            telemetry.addData("robotX", _hardware.odometry.getPosX(DistanceUnit.CM));
            telemetry.addData("robotY" ,_hardware.odometry.getPosY(DistanceUnit.CM));
            telemetry.addData("robotH", _hardware.odometry.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }

    boolean first = true;
    @Override
    public void run()
    {
        CommandScheduler.getInstance().run();
        _hardware.update();
        _hardware._chasis.setMovement(MovementCommand.robotVelocityX, MovementCommand.robotVelocityY, MovementCommand.robotVelocityW);
        _hardware._chasis.updateFieldCentric();
    }
}
