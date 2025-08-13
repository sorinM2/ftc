package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.WaitForTap;
import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Intake.Claw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialClaw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Intake.Intake;
import org.firstinspires.ftc.teamcode.core.Intake.V4b;
import org.firstinspires.ftc.teamcode.core.Lift;
import org.firstinspires.ftc.teamcode.core.OuttakeDifferential;

public class RetractExtendo extends SequentialCommandGroup {

    public RetractExtendo(Hardware hw)
    {

        this(hw, 600, DifferentialIntake.Rotation_ANGLE_INIT, DifferentialClaw.Rotation_ANGLE_SCAN);
                        //new InstantCommand(()->{hw.v4b.SetState(V4b.V4BStates.INIT);})

    }

    public RetractExtendo(Hardware hw, int delay)
    {
        this(hw, delay, DifferentialIntake.Rotation_ANGLE_INIT, DifferentialClaw.Rotation_ANGLE_SCAN);
    }

    public RetractExtendo(Hardware hw, int delay, double armPos, double clawPos)
    {
        super(
                new InstantCommand(()->{hw._lift.SetState(Lift.LiftStates.LOW_CHAMBER_RELEASE);}),
                new ConditionalCommand(new WaitForLift(200), new WaitCommand(1), ()->hw._extendo.getCurrentPosition() < 50),
                new InstantCommand(()->{
                    hw.differentialIntake.SetState(DifferentialIntake.IntakeDifferentialStates.TRANSFER);
                    hw.differentialClaw.SetState(DifferentialClaw.ClawDifferentialStates.TRANSFER);
                    hw._intake.SetState(Intake.IntakeState.Intake);
                    hw._clawOutake.openClaw();
                    hw.differential.SetState(OuttakeDifferential.DifferentialStates.INIT);
                }),
                //new WaitForLift(300),
                new InstantCommand(()->{hw._extendo.SetState(Extendo.ExtendoStates.TRANSFER);}),
                new ParallelRaceGroup(
                        new WaitForTap(hw),
                        new WaitCommand(delay)
                ),
                new InstantCommand(()->
                {
                    hw._clawOutake.closeClawSample();
                    hw._intake.SetState(Intake.IntakeState.Outake);
                }),
                new WaitCommand(160),
                new InstantCommand(()->{
                    hw._lift.SetState(Lift.LiftStates.HIGH_BASKET);
                }),
                //new WaitCommand(40),
                new InstantCommand(()->
                {
                    hw.differential.SetState(OuttakeDifferential.DifferentialStates.INTERMEDIATE);
                    hw.differentialIntake.SetState(DifferentialIntake.IntakeDifferentialStates.SCAN);
                }),
                new InstantCommand(()->{hw._extendo.SetState(Extendo.ExtendoStates.SCORING);}),
                new WaitCommand(10),
                new InstantCommand(()->
                {
                    hw._extendo.SetState(Extendo.ExtendoStates.INIT);
                    hw.differentialIntake.setPosition(DifferentialIntake.LINEAR_ANGLE_INIT, armPos);
                    hw.differentialClaw.setPosition(DifferentialClaw.LINEAR_ANGLE_PICKUP, clawPos);
                })
                //new InstantCommand(()->{hw.v4b.SetState(V4b.V4BStates.INIT);})
        );
    }
}
