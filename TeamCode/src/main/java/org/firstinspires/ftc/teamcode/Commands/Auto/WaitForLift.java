package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Lift;

import java.util.Collections;
import java.util.Set;

public class WaitForLift implements Command {
    public  double tolerance = 15;

    public WaitForLift(double tol)
    {
        tolerance = tol;
    }

    public WaitForLift()
    {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Lift.delta) < tolerance;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }
}
