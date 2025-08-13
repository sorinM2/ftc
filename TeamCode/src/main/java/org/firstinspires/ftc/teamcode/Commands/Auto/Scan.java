package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.Pickup;
import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialClaw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Intake.Intake;
import org.firstinspires.ftc.teamcode.core.Lift;

public class Scan extends SequentialCommandGroup {

    public static boolean ScanFinsihed = false;
    public  static double intakepos = 0.d, clawpos = 0.d;
    public static int extendoPos = 0;
    public static double YPOS = 0;
    public Scan(Hardware _hardware)
    {
        super(
                new InstantCommand(()->{
                    MovementCommand.parked = true;
                    _hardware._lift.SetState(Lift.LiftStates.CAMERA);
                    _hardware._extendo.SetState(Extendo.ExtendoStates.INIT);
                }),
                new WaitForExtendo(30),
                new WaitForLift(40),
                new WaitCommand(100),
                new InstantCommand(()->{
                    _hardware.camera.extendoPos = 0;
                    _hardware.camera.sampleY = 0;
                }),
                new WaitUntilCommand(()->{
                    if( !Double.isNaN(_hardware.camera.turretPos) && _hardware.camera.extendoPos != 0 && _hardware.camera.clawPos != 0)
                    {
                        extendoPos = _hardware.camera.extendoPos;
                        clawpos = _hardware.camera.clawPos;
                        intakepos = _hardware.camera.turretPos;
                        _hardware.camera.clawPos = 0;
                        YPOS = _hardware.camera.sampleY;
                        return true;
                    }
                    else return false;
                }),
                new InstantCommand(()->{_hardware._extendo.setPosition(extendoPos);}),
                new InstantCommand(()->{_hardware._intake.SetState(Intake.IntakeState.Outake);}),
                new InstantCommand(()->{_hardware.differentialIntake.setPosition(0.43, intakepos);}),
                new InstantCommand(()->{_hardware.differentialClaw.setPosition(DifferentialClaw.LINEAR_ANGLE_SCAN, clawpos);}),
                new WaitForExtendo(15),
                new WaitCommand(280),
                new PickupAuto(_hardware),
                new WaitCommand(40),
                new InstantCommand(()->{_hardware.differentialIntake.setPosition(DifferentialIntake.LINEAR_ANGLE_INIT, DifferentialIntake.Rotation_ANGLE_SCAN);
                                        _hardware.differentialClaw.SetState(DifferentialClaw.ClawDifferentialStates.SCAN);}),
                new WaitCommand(40),
                new InstantCommand(()->{ScanFinsihed = true;
                       })
        );
    }
}
