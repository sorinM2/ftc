package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;
import org.firstinspires.ftc.teamcode.Commands.Auto.WaitForAngle;
import org.firstinspires.ftc.teamcode.Commands.Auto.WaitForPosition;
import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Intake.Claw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialClaw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;
import org.firstinspires.ftc.teamcode.core.Intake.Intake;
import org.firstinspires.ftc.teamcode.core.Intake.V4b;

@Config
public class PickupAuto extends SequentialCommandGroup {
    public static int Sleep =20;
    public PickupAuto(Hardware hw)
    {
        super(
                new WaitForExtendo(15),
                new ParallelCommandGroup(
                        new InstantCommand(()-> {hw._intake.SetState(Intake.IntakeState.Outake);}),
                        new InstantCommand(()->{hw.differentialIntake.setPosition(DifferentialIntake.LINEAR_ANGLE_PICKUP, hw.differentialIntake.getRotationAngle());}),
                        new InstantCommand(()->{hw.differentialClaw.setPosition(DifferentialClaw.LINEAR_ANGLE_PICKUP, hw.differentialClaw.getRotationAngle());})
                ),
                new WaitCommand(20),
                new InstantCommand(()->{hw._intake.SetState(Intake.IntakeState.Intake);}),
                new WaitCommand(20)
        );
    }
}
