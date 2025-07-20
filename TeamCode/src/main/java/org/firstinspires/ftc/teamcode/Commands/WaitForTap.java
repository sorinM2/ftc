package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.core.Hardware;

import java.util.Collections;
import java.util.Set;

public class WaitForTap implements Command {

    Hardware _hardware;
    public WaitForTap(Hardware hw)
    {
        _hardware = hw;
    }

    @Override
    public boolean isFinished() {
        return !_hardware.transferDetection.getState();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }
}
