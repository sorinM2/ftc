package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Common.StaticVariables.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commands.Auto.ChangePosition;
import org.firstinspires.ftc.teamcode.Commands.Auto.Extend;
import org.firstinspires.ftc.teamcode.Commands.Auto.PickupAuto;
import org.firstinspires.ftc.teamcode.Commands.Auto.RetractExtendo;
import org.firstinspires.ftc.teamcode.Commands.Auto.ScoreHigh;
import org.firstinspires.ftc.teamcode.Commands.Auto.WaitForAngle;
import org.firstinspires.ftc.teamcode.Commands.Auto.WaitForExtendo;
import org.firstinspires.ftc.teamcode.Commands.Auto.WaitForLift;
import org.firstinspires.ftc.teamcode.Commands.IntakeExtend;
import org.firstinspires.ftc.teamcode.Commands.IntakeRetract;
import org.firstinspires.ftc.teamcode.Commands.Auto.MovementCommand;
import org.firstinspires.ftc.teamcode.Commands.Auto.WaitForPosition;
import org.firstinspires.ftc.teamcode.Commands.Pickup;
import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialClaw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Intake.Intake;
import org.firstinspires.ftc.teamcode.core.Lift;

@Config
@Autonomous
public class Positions extends CommandOpMode {
    SequentialCommandGroup testCommand;

    Hardware _hardware;
    MovementCommand movement;
    public static double xScore = 43, yScore = -28, hScore = 60;
    public static double xRight = 41, yRight = -30, hRight = 71; // sample right
    public static double xCenter = 44, yCenter = -22, hCenter = 90; // sample center
    public static double xLeft = 30, yLeft = -34, hLeft = 120; // sample left
    public static double rightClaw = 0.46;

    public  double intakepos = 0.d, clawpos = 0.d;
    public int extendoPos = 0;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        _hardware = new Hardware(hardwareMap, gamepad1, gamepad2, telemetry);
        _hardware.initialize();

        movement = new MovementCommand(_hardware);
        MovementCommand.parked = true;
        //MovementCommand.parked = true;
        CommandScheduler.getInstance().schedule(movement);

        testCommand = new SequentialCommandGroup(

        );


        CommandScheduler.getInstance().schedule(
                testCommand
        );
    }

    @Override
    public void run()
    {
        telemetry.addData("distanta sensor", _hardware.distSensor.getDistance(DistanceUnit.MM));

        CommandScheduler.getInstance().run();
        //_hardware.camera.getCoordinates(30, 30);
        //_hardware._extendo.setPosition(_hardware.camera.extendoPos);
        _hardware._controller1.update();
        _hardware.update();
        //_hardware._chasis.setMovement(MovementCommand.robotVelocityX, MovementCommand.robotVelocityY, MovementCommand.robotVelocityW);
        //_hardware._chasis.updateFieldCentric();
    }
}
