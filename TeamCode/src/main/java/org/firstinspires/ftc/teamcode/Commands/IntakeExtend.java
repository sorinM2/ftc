package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Intake.Claw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialClaw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Intake.Intake;
import org.firstinspires.ftc.teamcode.core.Intake.V4b;

public class IntakeExtend extends InstantCommand {

    public IntakeExtend(Hardware hw)
    {
        super(
                ()->{
                  hw._extendo.SetState(Extendo.ExtendoStates.EXTENDED);
//                  hw.v4b.SetState(V4b.V4BStates.SCANNING);
//                  hw._claw.SetState(Claw.ClawState.SCAN);
                    hw.differentialClaw.SetState(DifferentialClaw.ClawDifferentialStates.SCAN);
                    hw.differentialIntake.SetState(DifferentialIntake.IntakeDifferentialStates.SCAN);
                    hw._intake.SetState(Intake.IntakeState.Outake);
                }
        );
    }
}
