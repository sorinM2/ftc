package org.firstinspires.ftc.teamcode.Commands.Auto;

import static org.firstinspires.ftc.teamcode.Common.StaticVariables.telemetry;

import android.bluetooth.le.ScanSettings;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Lift;

import java.util.Collections;
import java.util.Set;

@Config
public class ScanSubmerssibleRepeat implements Command {

    public static double lastSensorDistance = 0.d;
    Scan scanCommand;
    private Hardware _hardware;

    boolean finished = false;

    public ScanSubmerssibleRepeat(Hardware hw)
    {
        _hardware = hw;
    }
    @Override
    public void initialize() {
        Scan.ScanFinsihed = false;
        scanCommand = new Scan(_hardware);
        scanCommand.initialize();
    }
    public static int tolerance = 20;
    @Override
    public void execute() {

        scanCommand.execute();

        if ( Scan.ScanFinsihed )
        {
            lastSensorDistance = _hardware.distSensor.getDistance(DistanceUnit.MM);
            if ( lastSensorDistance> tolerance )
            {
                _hardware._extendo.SetState(Extendo.ExtendoStates.INIT);
                _hardware.differentialIntake.SetState(DifferentialIntake.IntakeDifferentialStates.INIT);
                scanCommand = new Scan(_hardware);
                Scan.ScanFinsihed = false;
                scanCommand.initialize();
            }
            else finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }
}
