package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.core.Intake.Extendo;

import java.util.Collections;
import java.util.Set;

public class WaitForExtendo implements Command {
    public static double tolerance = 25;

    public WaitForExtendo()
    {
        tolerance = 25;
    }
    public WaitForExtendo(double tol)
    {
        tolerance = tol;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Extendo.delta) < tolerance;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }
}
