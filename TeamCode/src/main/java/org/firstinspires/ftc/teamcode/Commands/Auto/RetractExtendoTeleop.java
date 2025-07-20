package org.firstinspires.ftc.teamcode.Commands.Auto;

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

public class RetractExtendoTeleop extends SequentialCommandGroup {

    public RetractExtendoTeleop(Hardware hw)
    {

        this(hw, 700);
        //new InstantCommand(()->{hw.v4b.SetState(V4b.V4BStates.INIT);})

    }

    public RetractExtendoTeleop(Hardware hw, int delay)
    {
        super(
                new InstantCommand(()->{hw._lift.SetState(Lift.LiftStates.LOW_CHAMBER_RELEASE);}),
                new InstantCommand(()->{
                    hw.differentialIntake.SetState(DifferentialIntake.IntakeDifferentialStates.TRANSFER);
                    hw.differentialClaw.SetState(DifferentialClaw.ClawDifferentialStates.TRANSFER);
                    hw._intake.SetState(Intake.IntakeState.Intake);
                    hw._clawOutake.openClaw();
                    hw.differential.SetState(OuttakeDifferential.DifferentialStates.INIT);
                }),
                new WaitCommand(40),
                new InstantCommand(()->{hw._extendo.SetState(Extendo.ExtendoStates.TRANSFER);}),
                new ParallelRaceGroup(
                        new WaitForTap(hw),
                        new WaitCommand(delay)
                ),
                new InstantCommand(()->{
                    hw._intake.SetState(Intake.IntakeState.Outake);
                    hw._clawOutake.closeClawSample();
                }),
                new WaitCommand(40),
                new InstantCommand(()->
                {
                    hw.differential.SetState(OuttakeDifferential.DifferentialStates.INTERMEDIATE);
                    hw.differentialIntake.SetState(DifferentialIntake.IntakeDifferentialStates.INIT);
                })
                //new InstantCommand(()->{hw.v4b.SetState(V4b.V4BStates.INIT);})
        );
    }
}
