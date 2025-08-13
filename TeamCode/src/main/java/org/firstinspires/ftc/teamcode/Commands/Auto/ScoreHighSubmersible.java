package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Auto.PickupAuto;
import org.firstinspires.ftc.teamcode.Common.StaticVariables;
import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Intake.Claw;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Intake.Intake;
import org.firstinspires.ftc.teamcode.core.Intake.V4b;
import org.firstinspires.ftc.teamcode.core.Lift;
import org.firstinspires.ftc.teamcode.core.OuttakeDifferential;

public class ScoreHighSubmersible extends SequentialCommandGroup {
    public ScoreHighSubmersible(Hardware hw, double x, double y, double h, MovementCommand movement)
    {
        this(hw, x, y, h, movement, 230);
    }

    public ScoreHighSubmersible(Hardware hw, double x, double y, double h, MovementCommand movement, int liftDelay)
    {
        super(
                new InstantCommand(()->{
                    hw._lift.SetState(Lift.LiftStates.HIGH_BASKET_AUTO);
                    hw.differential.setPosition(OuttakeDifferential.LINEAR_ANGLE_BASKET, OuttakeDifferential.ROTATION_ANGLE_FORWARD);
//                    hw.v4b.SetState(V4b.V4BStates.SCANNING);
//                    hw._claw.SetState(Claw.ClawState.SCAN);
                }),
                new ParallelRaceGroup(
                         new ParallelCommandGroup(
                            new WaitForPosition(15),
                            new WaitForAngle(15)
                         ),
                        new WaitCommand(1700)

                        ),
                new WaitForLift(liftDelay),
                new InstantCommand(()->
                {
                    hw.differential.setPosition(OuttakeDifferential.LINEAR_ANGLE_BASKET_AUTO, OuttakeDifferential.ROTATION_ANGLE_FORWARD);
                }),
                new WaitCommand(70),
                new InstantCommand(()->
                {
                    hw._clawOutake.openClaw();
                }),
                new WaitCommand(100),
                new InstantCommand(()-> {
                    movement.angleWalk = true;
                    movement.pidDistance();
                }),
                new ChangePosition(x, y, h),
                new WaitCommand(300),
                new InstantCommand(()->{
                    hw._lift.SetState(Lift.LiftStates.LOW_CHAMBER_RELEASE);
                    hw._extendo.SetState(Extendo.ExtendoStates.INIT);
                    hw.differential.SetState(OuttakeDifferential.DifferentialStates.BASKET_INTERMEDIATE);
                    hw._intake.SetState(Intake.IntakeState.Outake);
                })

        );
    }
}
