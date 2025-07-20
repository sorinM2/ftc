package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Differential;
import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Hooks;
import org.firstinspires.ftc.teamcode.core.Intake.Claw;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Intake.V4b;
import org.firstinspires.ftc.teamcode.core.Lift;
import org.firstinspires.ftc.teamcode.core.OuttakeDifferential;
import org.firstinspires.ftc.teamcode.core.Pto;

import java.util.Collections;
import java.util.Set;

@Config
public class Hang implements Command {

    public enum HangStates {
        WAITING,
        DISABLED,
        READY_FOR_LVL2,
        HANG_LVL_2,
        KEEP_POSITION,
        GOING_TO_LVL3,
        READY_FOR_LVL3,
        HANG_LVL3,
        KEEP_POSITION_LVL3;
    }

    Hardware _hardware;
    public Hang(Hardware hw)
    {
        _hardware = hw;
        state = HangStates.READY_FOR_LVL2;
    }

    private HangStates state, nextState;

    private ElapsedTime timer = new ElapsedTime();
    private double waitingTime;

    public static double retractPower = 1, keepPower = 0.45;
    public static int liftRetractLvl2 = 250, liftRetractLvl3 = 0;

    @Override
    public void initialize() {
        //state = HangStates.DISABLED;
    }

    @Override
    public void execute() {
            switch (state) {
                case WAITING:
                    if (timer.seconds() > waitingTime) {
                        state = nextState;
                    }

                    break;

                case READY_FOR_LVL2:
                    _hardware._lift.SetState(Lift.LiftStates.HANG_LVL2);
                    _hardware._hooks.SetState(Hooks.HooksState.INTERMEDIATE);
//                    _hardware._claw.SetState(Claw.ClawState.FENCE);
//                    _hardware.v4b.SetState(V4b.V4BStates.FENCE);
                    _hardware.differentialIntake.setPosition(0.015, 0.06);
                    _hardware.differentialClaw.setPosition(0.51, 0.28);
                    _hardware.differential.SetState(OuttakeDifferential.DifferentialStates.FENCE);

                    retractPower = 0.7;

                    if (_hardware._lift.getCurrentPosition() >= Lift.HANG_LVL2 - 50) {
                        _hardware._pto.SetState(Pto.PTOStates.ACTIVATED);
                        _hardware._lift.disablePID();

                        state = HangStates.WAITING;
                        nextState = HangStates.HANG_LVL_2;
                        waitingTime = 0.3; timer.reset();
                    }

                    break;


                case HANG_LVL_2:
                    _hardware._chasis.updateHang(retractPower);
                    _hardware._lift.setPower(-retractPower / 2);

                    if (_hardware._lift.getCurrentPosition() <= liftRetractLvl2) {
                        _hardware._hooks.SetState(Hooks.HooksState.ACTIVATED);

                        state = HangStates.KEEP_POSITION;
                        timer.reset(); waitingTime = 0.1;
                    }

                    break;

                case KEEP_POSITION:
                    _hardware._chasis.updateHang(keepPower);
                    _hardware._lift.setPower(-keepPower / 2);

                    if (timer.seconds() > waitingTime) {
                        _hardware._lift.setPower(0);
                        _hardware._chasis.updateHang(0);
                        _hardware._pto.SetState(Pto.PTOStates.INIT);

                        state = HangStates.GOING_TO_LVL3;
                        timer.reset(); waitingTime = 0.1;
                    }

                    break;

                case GOING_TO_LVL3:
                    if (timer.seconds() < waitingTime) break;

                    _hardware._lift.SetState(Lift.LiftStates.HANG_LVL3); _hardware._lift.enablePID();
                    _hardware._hooks.SetState(Hooks.HooksState.READY_LVL_3);

                    if (_hardware._lift.getCurrentPosition() >= 680) {
                        state = HangStates.READY_FOR_LVL3;
                        //extendo.setPosition(-100);
                    }

                    break;

                case READY_FOR_LVL3:
                    _hardware._pto.SetState(Pto.PTOStates.ACTIVATED);
                    _hardware._hooks.SetState(Hooks.HooksState.INIT);
                    _hardware._lift.disablePID();

                    state = HangStates.HANG_LVL3; retractPower = 1;
                    timer.reset(); waitingTime = 0.3;

                    break;

                case HANG_LVL3:
                    if (timer.seconds() < waitingTime) break;

                    if (timer.seconds() > 1)
                        _hardware._extendo.SetState(Extendo.ExtendoStates.INIT);

                    _hardware._chasis.updateHang(retractPower);
                    _hardware._lift.setPower(-retractPower / 2);

                    if (_hardware._lift.getCurrentPosition() < 220) {
                        retractPower = 0.6;
                        //extendo.setPosition(-20);
                    }

                    if (_hardware._lift.getCurrentPosition() <= liftRetractLvl3) {
                        state = HangStates.KEEP_POSITION_LVL3;
                        timer.reset(); waitingTime = 0.5;
                    }

                    break;

                case KEEP_POSITION_LVL3:
                    _hardware._chasis.updateHang(0.3);
                    _hardware._lift.setPower(-0.3 / 2);

                    break;
            }
        }

    @Override
    public void end(boolean interrupted) {
        Command.super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return Command.super.isFinished();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }
}
