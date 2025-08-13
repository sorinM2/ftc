package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;

public class ResetLift extends SequentialCommandGroup {
    public ResetLift(Hardware _hardware)
    {
        super(
            new InstantCommand(()->{_hardware._lift.setPower(-0.75);
                _hardware._lift.disablePID();
            }),
            new ParallelRaceGroup(
                    new WaitUntilCommand(()->
                            _hardware._lift.motorDown.getCurrent(CurrentUnit.AMPS) > 6 ||
                                    _hardware._lift.motorUp.getCurrent(CurrentUnit.AMPS) > 6
                    ),
                    new WaitCommand(1000)
            ),
            new InstantCommand(()->{
                _hardware._lift.setPower(0.0);

                _hardware._lift.motorDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                _hardware._lift.motorDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                _hardware._lift.motorUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                _hardware._lift.motorUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                _hardware._lift.enablePID();
            }),
            new InstantCommand(()->{
                _hardware._extendo.disablePID();
                _hardware._extendo.motor.setPower(-1);
            }),
            new ParallelRaceGroup(
                    new WaitUntilCommand(()->
                            _hardware._extendo.motor.getCurrent(CurrentUnit.AMPS) > 8.5
                    ),
                    new WaitCommand(1500)
            ),
            new InstantCommand(()->{
                _hardware._extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                _hardware._extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                _hardware._extendo.enablePID();
                _hardware._extendo.motor.setPower(0);
                _hardware.differentialIntake.SetState(DifferentialIntake.IntakeDifferentialStates.TRANSFER);
            })
        );
    }
}
