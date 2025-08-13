package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Intake.Claw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialClaw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Intake.Intake;
import org.firstinspires.ftc.teamcode.core.Intake.V4b;

public class Extend extends InstantCommand {

    public Extend(Hardware hw, int epos)
    {
        super(
                  ()-> {
                      hw._extendo.SetState(Extendo.ExtendoStates.SCORING);
                      hw._extendo.setPosition(epos);
                      //hw.v4b.SetState(V4b.V4BStates.SCANNING);
                      //hw._claw.SetState(Claw.ClawState.SCAN);
//                      hw.differentialIntake.SetState(DifferentialIntake.IntakeDifferentialStates.SCAN);
//                      hw.differentialClaw.SetState(DifferentialClaw.ClawDifferentialStates.SCAN);
                      hw._intake.SetState(Intake.IntakeState.Outake);
                  }
        );
    }
}
