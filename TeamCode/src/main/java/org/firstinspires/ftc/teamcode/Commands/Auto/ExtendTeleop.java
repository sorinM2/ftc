package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Intake.Claw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialClaw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Intake.Intake;
import org.firstinspires.ftc.teamcode.core.Intake.V4b;

public class ExtendTeleop extends SequentialCommandGroup {

    public ExtendTeleop(Hardware hw, int epos)
    {
        super(
                new InstantCommand(
                    ()-> {
                        hw._extendo.SetState(Extendo.ExtendoStates.EXTENDED);
                        //hw.v4b.SetState(V4b.V4BStates.SCANNING);
                        //hw._claw.SetState(Claw.ClawState.SCAN);
                        hw.differentialIntake.setPosition(0.38, DifferentialIntake.Rotation_ANGLE_SCAN);
                        hw.differentialClaw.SetState(DifferentialClaw.ClawDifferentialStates.SCAN);
                        hw._intake.SetState(Intake.IntakeState.Outake);
                    }
                ),
                new WaitForExtendo(30),
                new InstantCommand(()->{
                    hw.differentialIntake.SetState(DifferentialIntake.IntakeDifferentialStates.SCAN);
                })
        );
    }
}
