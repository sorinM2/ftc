package org.firstinspires.ftc.teamcode.opmodes;

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
import org.firstinspires.ftc.teamcode.Commands.IntakeExtend;
import org.firstinspires.ftc.teamcode.Commands.IntakeRetract;
import org.firstinspires.ftc.teamcode.Commands.Auto.MovementCommand;
import org.firstinspires.ftc.teamcode.Commands.Auto.WaitForPosition;
import org.firstinspires.ftc.teamcode.Commands.Pickup;
import org.firstinspires.ftc.teamcode.Commands.Auto.PickupAuto;
import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialClaw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Lift;
import org.firstinspires.ftc.teamcode.core.OuttakeDifferential;

@Config
@Autonomous
public class Auto extends CommandOpMode {
    SequentialCommandGroup testCommand;

    Hardware _hardware;
    MovementCommand movement;
    public static double xScore = 43, yScore = -30, hScore = 65;

    public static double xRight = 51, yRight = -30, hRight = 62, intakeRotRight = 0.1; // sample right
    public static double xCenter = 56, yCenter = -35, hCenter = 89, intakeRotCenter = -0.05d; // sample center
    public static double xLeft = 56, yLeft = -35, hLeft = 89, intakeRotLeft = 0.31d; // sample left
    public static double xSub = -43, ySub = -152, hSub = 0; // sample left
    public static double xMid = -35, yMid = -152, hMid =0; // sample left

    public static double rightClaw = 0.27;
    public static double centerClaw = 0.26;
    public static double leftClaw = 0.37;

    public static double rightDiff = 0.295;
    public static double centerDiff = 0.13;
    public static double leftDiff = 0.13;

    public static int rightExtendo = 145;
    public static int centreExtendo = 125;
    public static int leftExtendo = 150;

    @Override
    public void initialize() {
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
                    _hardware.differentialIntake.setPosition(DifferentialIntake.LINEAR_ANGLE_SCAN, intakeRotRight);
                    _hardware.differentialClaw.setPosition(DifferentialClaw.LINEAR_ANGLE_SCAN, rightClaw);
                }),
                new ScoreHigh(_hardware, xRight, yRight, hRight, rightDiff, 1000),

                new ChangePosition(xCenter, yCenter, hCenter),
                new RetractExtendo(_hardware),
                new Extend(_hardware, centreExtendo),
                new InstantCommand(()->{_hardware.differentialIntake.setPosition(DifferentialIntake.LINEAR_ANGLE_SCAN, intakeRotCenter);
                _hardware.differentialClaw.setPosition(DifferentialClaw.LINEAR_ANGLE_SCAN, centerClaw);}),
                new ScoreHigh(_hardware, xCenter, yCenter, hCenter, rightDiff, 400),

                new ChangePosition(xCenter, yCenter, hCenter ),
                new RetractExtendo(_hardware),
                new Extend(_hardware, leftExtendo),
                new WaitForExtendo(60),
                new InstantCommand(()->{_hardware.differentialIntake.setPosition(DifferentialIntake.LINEAR_ANGLE_SCAN, intakeRotLeft);
                    _hardware.differentialClaw.setPosition(DifferentialClaw.LINEAR_ANGLE_SCAN, leftClaw); }),
                new ScoreHigh(_hardware, xLeft, yLeft, hLeft, centerDiff, 400),

                new ChangePosition(xCenter, yCenter, hCenter),
                new RetractExtendo(_hardware),
                new ScoreHighSubmersible(_hardware, xMid, yMid, hMid, movement)//,
//
//                new WaitForPosition(60),
//                new InstantCommand(()-> {movement.angleWalk = false;}),
//                new ChangePosition(xSub, ySub, hSub),
//                //new WaitForPosition(25),
//                new ScanSubmerssibleRepeat(_hardware),
//                new ParallelCommandGroup(
//                new RetractExtendo(_hardware, 1350),
//                new ChangePosition(xScore - 2, yScore, hScore),
//                new InstantCommand(()-> {movement.angleWalk = true;})),
//                new WaitForPosition(80),
//                new InstantCommand(()-> {movement.angleWalk = false;}),
//                new ScoreHighSubmersible(_hardware, xMid, yMid, hMid, movement),
//
//                new InstantCommand(()-> {movement.angleWalk = true;}),
//                new WaitForPosition(60),
//                new InstantCommand(()-> {movement.angleWalk = false;}),
//                new ChangePosition(xSub - 5, ySub, hSub),
//                //new WaitForPosition(25),
//                new ScanSubmerssibleRepeat(_hardware),
//                new ParallelCommandGroup(
//                new RetractExtendo(_hardware, 1350),
//                new ChangePosition(xScore - 2, yScore, hScore),
//                new InstantCommand(()-> {movement.angleWalk = true;})),
//                new WaitForPosition(80),
//                new InstantCommand(()-> {movement.angleWalk = false;}),
//                new ScoreHighSubmersible(_hardware, xMid, yMid, hMid, movement),
//
//                new InstantCommand(()-> {movement.angleWalk = true;}),
//                new WaitForPosition(60),
//                new InstantCommand(()-> {movement.angleWalk = false;}),
//                new ChangePosition(xSub -2, ySub, hSub + 5),
//                //new WaitForPosition(25),
//                new ScanSubmerssibleRepeat(_hardware),
//                new ParallelCommandGroup(
//                new RetractExtendo(_hardware, 1350),
//                new ChangePosition(xScore, yScore, hScore),
//                new InstantCommand(()-> {movement.angleWalk = true;})),
//                new WaitForPosition(80),
//                new InstantCommand(()-> {movement.angleWalk = false;}),
//                new ScoreHighSubmersible(_hardware, xMid, yMid, hMid, movement),
//
//                new InstantCommand(()-> {movement.angleWalk = true;}),
//                new WaitForPosition(60),
//                new InstantCommand(()-> {movement.angleWalk = false;}),
//                new ChangePosition(xSub - 10, ySub , hSub + 5),
//                //new WaitForPosition(25),
//                new ScanSubmerssibleRepeat(_hardware),
//                new ParallelCommandGroup(
//                new RetractExtendo(_hardware, 1350),
//                new ChangePosition(xScore- 3, yScore - 2, hScore),
//                new InstantCommand(()-> {movement.angleWalk = true;})),
//                new WaitForPosition(80),
//                new InstantCommand(()-> {movement.angleWalk = false;}),
//                new ScoreHighSubmersible(_hardware, xMid, yMid, hMid, movement),
//
//                new InstantCommand(()-> {movement.angleWalk = true;}),
//                new WaitForPosition(60),
//                new InstantCommand(()-> {movement.angleWalk = false;}),
//                new ChangePosition(xSub -4, ySub - 2, hSub + 10),
//                //new WaitForPosition(25),
//                new ScanSubmerssibleRepeat(_hardware),
//                new ParallelCommandGroup(
//                new RetractExtendo(_hardware, 1350),
//                new ChangePosition(xScore, yScore, hScore),
//                new InstantCommand(()-> {movement.angleWalk = true;})),
//                new WaitForPosition(80),
//                new InstantCommand(()-> {movement.angleWalk = false;}),
//                new ScoreHighSubmersible(_hardware, xMid, yMid, hMid, movement)
        );


        CommandScheduler.getInstance().schedule(
                testCommand
        );
    }

    @Override
    public void run()
    {
        CommandScheduler.getInstance().run();
        _hardware.update();
        _hardware._chasis.setMovement(MovementCommand.robotVelocityX, MovementCommand.robotVelocityY, MovementCommand.robotVelocityW);
        _hardware._chasis.updateFieldCentric();
    }
}
