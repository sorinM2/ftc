package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Lift;
import org.firstinspires.ftc.teamcode.core.OuttakeDifferential;

public class PickupAfter extends ParallelCommandGroup {
    public PickupAfter(Hardware hw)
    {
        super(
            new PickupAuto(hw),
            new InstantCommand(()->{
                hw._lift.SetState(Lift.LiftStates.LOW_CHAMBER_RELEASE);
                hw.differential.SetState(OuttakeDifferential.DifferentialStates.INIT);
            })
        );
    }
}
