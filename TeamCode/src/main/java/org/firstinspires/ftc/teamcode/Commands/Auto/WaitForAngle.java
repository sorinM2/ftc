package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.Collections;
import java.util.Set;

public class WaitForAngle implements Command {

    double tolerance = 5.f;
    public WaitForAngle(double tol)
    {
        tolerance = tol;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(MovementCommand.deltaAngle) < tolerance;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }
}
