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
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialClaw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Intake.V4b;
import org.firstinspires.ftc.teamcode.core.Lift;
import org.firstinspires.ftc.teamcode.core.OuttakeDifferential;

public class ScoreHigh extends SequentialCommandGroup {
    public ScoreHigh(Hardware hw, double x, double y, double h, double diff, int wait)
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
                        new WaitForPosition(6),
                        new WaitForAngle(5)
                    ),
                    new WaitCommand(wait)
                ),
                new WaitForLift(350),
                new InstantCommand(()->
                {
                    hw.differential.setPosition(OuttakeDifferential.LINEAR_ANGLE_BASKET_AUTO, OuttakeDifferential.ROTATION_ANGLE_FORWARD);
                }),
                new WaitForLift(30),
                new ParallelCommandGroup(
                    new InstantCommand(()->
                    {
                        hw._clawOutake.openClaw();
                    }),
                    new PickupAuto(hw)

                ),
                new WaitCommand(70),
                new InstantCommand(()->{hw.differential.setPosition(OuttakeDifferential.LINEAR_ANGLE_INIT, OuttakeDifferential.ROTATION_ANGLE_INIT);
                    hw._lift.SetState(Lift.LiftStates.AUTO_TRANSFER);
                })
        );
    }
}
