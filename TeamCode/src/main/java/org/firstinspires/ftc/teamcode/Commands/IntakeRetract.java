package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Intake.Claw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialClaw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Intake.Intake;
import org.firstinspires.ftc.teamcode.core.Intake.V4b;
import org.firstinspires.ftc.teamcode.core.OuttakeDifferential;

public class IntakeRetract extends SequentialCommandGroup {
    public IntakeRetract(Hardware hw)
    {
        super(
                        new InstantCommand(()->{
                            hw._extendo.SetState(Extendo.ExtendoStates.TRANSFER);
                            hw.differentialIntake.SetState(DifferentialIntake.IntakeDifferentialStates.TRANSFER);
                            hw.differentialClaw.SetState(DifferentialClaw.ClawDifferentialStates.TRANSFER);
                            hw._intake.SetState(Intake.IntakeState.Intake);
                            hw._clawOutake.openClaw();
                            hw.differential.SetState(OuttakeDifferential.DifferentialStates.INIT);
                        }),
                        new WaitForTap(hw),
                        new WaitCommand(100),
                        new InstantCommand(()->{
                            hw._intake.SetState(Intake.IntakeState.Intermediate);
                            hw._clawOutake.closeClawSample();
                        }),

                        new InstantCommand(()->{
                            hw._intake.SetState(Intake.IntakeState.Intermediate);
                        }),
                        new WaitCommand(150),
                        new InstantCommand(()->{
                            hw.differential.SetState(OuttakeDifferential.DifferentialStates.INTERMEDIATE);
                            hw.differentialIntake.SetState(DifferentialIntake.IntakeDifferentialStates.INIT);
                        })
        );
    }
}
