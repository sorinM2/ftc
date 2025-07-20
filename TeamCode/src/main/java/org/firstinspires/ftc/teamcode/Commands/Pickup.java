package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Intake.Claw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialClaw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;
import org.firstinspires.ftc.teamcode.core.Intake.Intake;
import org.firstinspires.ftc.teamcode.core.Intake.V4b;

@Config
public class Pickup extends SequentialCommandGroup {
    public static int Sleep = 130;
    public Pickup(Hardware hw)
    {
        super(
                new InstantCommand(()-> {hw._intake.SetState(Intake.IntakeState.Outake);}),
                new WaitCommand(100),
                new ParallelCommandGroup(
                new InstantCommand(()->{hw.differentialIntake.setPosition(DifferentialIntake.LINEAR_ANGLE_PICKUP, hw.differentialIntake.getRotationAngle());}),
                        new InstantCommand(()->{hw.differentialClaw.setPosition(DifferentialClaw.LINEAR_ANGLE_PICKUP, hw.differentialClaw.getRotationAngle());})
                ),
                new WaitCommand(Sleep),
                new InstantCommand(()->{hw._intake.SetState(Intake.IntakeState.Intake);}),
                new WaitCommand(Sleep),
                new ParallelCommandGroup(
                        new InstantCommand(()->{hw.differentialIntake.setPosition(DifferentialIntake.LINEAR_ANGLE_SCAN, hw.differentialIntake.getRotationAngle());}),
                        new InstantCommand(()->{hw.differentialClaw.setPosition(DifferentialClaw.LINEAR_ANGLE_PICKUP, hw.differentialClaw.getRotationAngle());})
                ),
                new WaitCommand(Sleep)

                );
    }
}
