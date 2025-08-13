package org.firstinspires.ftc.teamcode.core;


import static org.firstinspires.ftc.teamcode.Common.StaticVariables.gamepad1;
import static org.firstinspires.ftc.teamcode.Common.StaticVariables.gamepad2;
import static org.firstinspires.ftc.teamcode.Common.StaticVariables.telemetry;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Commands.Auto.Extend;
import org.firstinspires.ftc.teamcode.Commands.Auto.RetractExtendo;
import org.firstinspires.ftc.teamcode.Commands.Auto.RetractExtendoTeleop;
import org.firstinspires.ftc.teamcode.Commands.Hang;
import org.firstinspires.ftc.teamcode.Commands.IntakeExtend;
import org.firstinspires.ftc.teamcode.Commands.IntakeRetract;
import org.firstinspires.ftc.teamcode.Commands.Pickup;
import org.firstinspires.ftc.teamcode.Commands.ResetLift;
import org.firstinspires.ftc.teamcode.core.Intake.Claw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialClaw;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Intake.Intake;


public class Controller2 extends Controller{

    Hardware _hardware;
    public Controller2(Hardware hw)
    {
        _hardware = hw;
    }
    public static double Krotation = 0.025;
    public static double Klift = 10;

    public static double clawAngle = 0.345d;
    public static boolean lowBasket = false;
    @Override
    public void initialize() {
        lowBasket = false;
        gamepad = new GamepadEx(gamepad2);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(()->{
                    if ( _hardware._lift.state == Lift.LiftStates.HIGH_BASKET || _hardware._lift.state == Lift.LiftStates.LOW_BASKET)
                        _hardware.differential.setPosition(_hardware.differential.getLinearAngle(), OuttakeDifferential.ROTATION_ANGLE_REVERSE);
                })
        );



        gamepad.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new ResetLift(_hardware)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(()->{
                    if ( _hardware._lift.state == Lift.LiftStates.HIGH_BASKET || _hardware._lift.state == Lift.LiftStates.LOW_BASKET)
                        _hardware.differential.setPosition(_hardware.differential.getLinearAngle(), OuttakeDifferential.ROTATION_ANGLE_FORWARD);
                })
        );        gamepad.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
                new InstantCommand(()->{
                    lowBasket = true;
                }),
                new InstantCommand(()->{
                    lowBasket = false;
                })
        );


    }
    public static double Kextendo = 80;

    @Override
    public void update() {
        telemetry.addData("amps",  _hardware._extendo.motor.getCurrent(CurrentUnit.AMPS));

//        if (gamepad2.right_trigger > 0.1) {
//            _hardware.differentialClaw.setPosition(_hardware.differentialClaw.getLinearAngle() + 0.005, _hardware.differentialClaw.getRotationAngle());
//        }
//
//        if (gamepad2.left_trigger > 0.1) {
//            _hardware.differentialClaw.setPosition(_hardware.differentialClaw.getLinearAngle() - 0.005, _hardware.differentialClaw.getRotationAngle());
//        }

        if (  gamepad2.right_stick_x > 0.2 )
        {
            _hardware.differentialClaw.setPosition(_hardware.differentialClaw.getLinearAngle(), _hardware.differentialClaw.getRotationAngle() + 0.005);
            //_hardware.differentialClaw.setPosition(_hardware.differentialClaw.getLinearAngle(), DifferentialClaw.Rotation_ANGLE_INIT + _hardware.differentialIntake.getRotationAngle() / 0.0028 / 180 / 4);
        }

        if (  gamepad2.right_stick_x < -0.2 )
        {
            _hardware.differential.setPosition(_hardware.differential.getLinearAngle() - 0.005, _hardware.differential.getRotationAngle());
            //_hardware.differentialClaw.setPosition(_hardware.differentialClaw.getLinearAngle(), DifferentialClaw.Rotation_ANGLE_INIT + _hardware.differentialIntake.getRotationAngle() / 0.0028 / 180 / 4);
        }

        if (  gamepad2.right_stick_y > 0.2 )
        {
            //clawAngle += 0.1;
            _hardware.differential.setPosition(_hardware.differential.getLinearAngle() + 0.005, _hardware.differential.getRotationAngle());
        }

        if (  gamepad2.right_stick_y < -0.2 )
        {
            //clawAngle -= 0.1;
            _hardware.differentialClaw.setPosition(_hardware.differentialClaw.getLinearAngle() , _hardware.differentialClaw.getRotationAngle() - 0.005);
        }

        //claw differential

//        if (  gamepad1.right_trigger > 0.2 )
//        {
//            _hardware.differentialClaw.setPosition(_hardware.differentialClaw.getLinearAngle(), _hardware.differentialClaw.getRotationAngle() + 0.005);
//        }
//
//        if (  gamepad1.left_trigger > 0.2 )
//        {
//            _hardware.differentialClaw.setPosition(_hardware.differentialClaw.getLinearAngle(), _hardware.differentialClaw.getRotationAngle() - 0.005);
//        }

        if (  gamepad2.left_stick_y > 0.2 )
        {
            _hardware.differentialIntake.setPosition(_hardware.differentialIntake.getLinearAngle() + 0.005, _hardware.differentialIntake.getRotationAngle() );
        }

        if (  gamepad2.left_stick_y < -0.2 )
        {
            _hardware.differentialIntake.setPosition(_hardware.differentialIntake.getLinearAngle() - 0.005, _hardware.differentialIntake.getRotationAngle() );
        }
        telemetry.addData("intake linear angle", _hardware.differentialIntake.getLinearAngle());
        telemetry.addData("intake rotation angle", _hardware.differentialIntake.getRotationAngle());
        telemetry.addData("claw linear angle", _hardware.differentialClaw.getLinearAngle());
        telemetry.addData("claw rotation angle", _hardware.differentialClaw.getRotationAngle());
        telemetry.addData("diff rotation", _hardware.differential.getRotationAngle());
        telemetry.addData("claw rotation angle", _hardware.differentialClaw.getRotationAngle());

    }

    @Override
    void shutdown() {

    }

    public Controller2()
    {

    }
}