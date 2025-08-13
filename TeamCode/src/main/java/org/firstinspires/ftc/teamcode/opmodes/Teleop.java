package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ResetLift;
import org.firstinspires.ftc.teamcode.Common.StaticVariables;
import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Pto;

@TeleOp
public class Teleop extends CommandOpMode {

    Hardware _hardware;



    @Override
    public void initialize() {

        StaticVariables.teleOp = true;
        CommandScheduler.getInstance().reset();
        _hardware = new Hardware(hardwareMap, gamepad1, gamepad2, telemetry);
        _hardware.initialize();
        CommandScheduler.getInstance().schedule(new ResetLift(_hardware));

    }

    @Override
    public void run()
    {
        CommandScheduler.getInstance().run();
        _hardware.update();
        _hardware._controller1.update();
        if ( _hardware._pto.state == Pto.PTOStates.INIT )
            _hardware._chasis.updateFieldCentric();
    }

}
