package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.Collections;
import java.util.Set;

public class WaitForPosition implements Command {

    double tolerance = 1.f;
    public WaitForPosition(double tol)
    {
        tolerance = tol;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(MovementCommand.deltaDistance) < tolerance;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }
}
