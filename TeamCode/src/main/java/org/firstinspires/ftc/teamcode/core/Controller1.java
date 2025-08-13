package org.firstinspires.ftc.teamcode.core;


import static org.firstinspires.ftc.teamcode.Common.StaticVariables.gamepad1;
import static org.firstinspires.ftc.teamcode.Common.StaticVariables.gamepad2;
import static org.firstinspires.ftc.teamcode.Common.StaticVariables.telemetry;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.Auto.Extend;
import org.firstinspires.ftc.teamcode.Commands.Auto.ExtendTeleop;
import org.firstinspires.ftc.teamcode.Commands.Auto.RetractExtendo;
import org.firstinspires.ftc.teamcode.Commands.Auto.RetractExtendoTeleop;
import org.firstinspires.ftc.teamcode.Commands.Hang;
import org.firstinspires.ftc.teamcode.Commands.IntakeExtend;
import org.firstinspires.ftc.teamcode.Commands.IntakeRetract;
import org.firstinspires.ftc.teamcode.Commands.Pickup;
import org.firstinspires.ftc.teamcode.core.Intake.Claw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialClaw;
import org.firstinspires.ftc.teamcode.core.Intake.DifferentialIntake;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;
import org.firstinspires.ftc.teamcode.core.Intake.Intake;


public class Controller1 extends Controller{

    Hardware _hardware;
    public Controller1(Hardware hw)
    {
        _hardware = hw;
    }
    public static double Krotation = 0.025;
    public static double Klift = 10;

    public static double clawAngle = 0.345d;
    @Override
    public void initialize() {
        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.START).whenPressed(new InstantCommand(()->{
            _hardware._chasis.imu.resetYaw();
        }));

        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(()->
        {
            _hardware._intake.SetState(Intake.IntakeState.Outake);
        }));



        telemetry.addData("extendo:", _hardware._extendo.getCurrentPosition());
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).toggleWhenPressed(
                new InstantCommand(()->{Controller2.lowBasket = true;}),
                new InstantCommand(()->{Controller2.lowBasket = false;})
        );

                gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(
                new InstantCommand(()->
                {
                    if ( Controller2.lowBasket) {
                        _hardware._lift.SetState(Lift.LiftStates.LOW_BASKET);
                        _hardware._extendo.SetState(Extendo.ExtendoStates.INIT);
                    }
                    else {
                        _hardware._lift.SetState(Lift.LiftStates.HIGH_BASKET);
                        _hardware._extendo.SetState(Extendo.ExtendoStates.SCORING);
                    }
                    _hardware.differential.SetState(OuttakeDifferential.DifferentialStates.BASKET);
                }),
                new InstantCommand(()->
                {
                    _hardware._extendo.SetState(Extendo.ExtendoStates.INIT);
                    _hardware._lift.SetState(Lift.LiftStates.INIT);
                    _hardware.differential.SetState(OuttakeDifferential.DifferentialStates.INTERMEDIATE);
                }));



        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new InstantCommand(()->
                {
                    if ( _hardware._clawOutake.open)
                        _hardware._clawOutake.closeClawSample();
                    else
                        _hardware._clawOutake.openClaw();
                })
        );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new Pickup(_hardware));

        //gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new Hang(_hardware));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(()->{
                    if ( _hardware._lift.state == Lift.LiftStates.HIGH_BASKET || _hardware._lift.state == Lift.LiftStates.LOW_BASKET)
                        _hardware.differential.setPosition(_hardware.differential.getLinearAngle(), OuttakeDifferential.ROTATION_ANGLE_REVERSE);
                })
        );
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(()->{
                    if ( _hardware._lift.state == Lift.LiftStates.HIGH_BASKET || _hardware._lift.state == Lift.LiftStates.LOW_BASKET)
                        _hardware.differential.setPosition(_hardware.differential.getLinearAngle(), OuttakeDifferential.ROTATION_ANGLE_FORWARD);
                })
        );

        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new Hang(_hardware)
        );




    }
    public static double Kextendo = 80;

    boolean right_stick = false;
    boolean last_right_stick = false;
    @Override
    public void update() {

        right_stick = gamepad1.right_stick_button;



        if ( right_stick && !last_right_stick && !(_hardware._lift.state == Lift.LiftStates.HIGH_BASKET))
        {
            if ( _hardware._extendo.state != Extendo.ExtendoStates.EXTENDED)
                CommandScheduler.getInstance().schedule(new ExtendTeleop(_hardware, Extendo.EXTENDED));
            else CommandScheduler.getInstance().schedule(new RetractExtendoTeleop(_hardware));
        }
        else if ( right_stick && !last_right_stick )
        {
            CommandScheduler.getInstance().schedule(
                new InstantCommand(()->{
                    if ( _hardware._lift.state != Lift.LiftStates.HIGH_BASKET )
                        return;
                    _hardware._lift.setPosition(Lift.HANG_LVL3  );
                    if ( _hardware.differential.state == OuttakeDifferential.DifferentialStates.BASKET )
                        _hardware.differential.SetState(OuttakeDifferential.DifferentialStates.BASKET_AUTO);
                    else _hardware.differential.SetState(OuttakeDifferential.DifferentialStates.BASKET);
                })
            );
        }

        if ( _hardware._lift.state == Lift.LiftStates.HIGH_BASKET ) {
            _hardware._extendo.SetState(Extendo.ExtendoStates.SCORING);
            if ( _hardware.differential.state == OuttakeDifferential.DifferentialStates.INTERMEDIATE)
                _hardware.differential.SetState(OuttakeDifferential.DifferentialStates.BASKET);
        }

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double w = gamepad1.right_stick_x;
        _hardware._chasis.setMovement(x, y, w);

//        if (gamepad2.right_trigger > 0.1 ) {
//            _hardware.differentialClaw.setPosition(_hardware.differentialClaw.getLinearAngle() + 0.005, _hardware.differentialClaw.getRotationAngle());
//        }
//
//        if (gamepad2.left_trigger > 0.1) {
//            _hardware.differentialClaw.setPosition(_hardware.differentialClaw.getLinearAngle() - 0.005, _hardware.differentialClaw.getRotationAngle());
//        }


        telemetry.addData("test claw", clawAngle);
        telemetry.addData("claw test result", 0.345 - ( clawAngle - 90) / 700);
//        if (Math.abs(gamepad2.right_stick_x) > 0.2 && (_hardware._claw.state == Claw.ClawState.SCAN || _hardware._claw.state == Claw.ClawState.PICK_UP)) {
//            double intakeClawRotation = _hardware._claw.getClawRotation();
//            intakeClawRotation += gamepad2.right_stick_x * Krotation;
//
//            _hardware._claw.setClawRotation(intakeClawRotation);
//        }
//
        if (gamepad1.left_trigger > 0.2 && _hardware._extendo.state == Extendo.ExtendoStates.EXTENDED) {
            _hardware.differentialClaw.setPosition(_hardware.differentialClaw.getLinearAngle(), _hardware.differentialClaw.getRotationAngle() - 0.009);
        }
        else if (gamepad1.left_trigger > 0.2)
        {
            _hardware._lift.setPosition(_hardware._lift.targetPosition - (int)(gamepad1.left_trigger * Klift));
        }

        if (gamepad1.right_trigger > 0.2 && _hardware._extendo.state == Extendo.ExtendoStates.EXTENDED ) {
            _hardware.differentialClaw.setPosition(_hardware.differentialClaw.getLinearAngle(), _hardware.differentialClaw.getRotationAngle() + 0.009);
        }
        else if (gamepad1.right_trigger > 0.2)
        {
            _hardware._lift.setPosition(_hardware._lift.targetPosition + (int)(gamepad1.right_trigger * Klift));
        }

        if ( gamepad2.left_trigger > 0.2 )
        {
            _hardware._extendo.setPosition(_hardware._extendo.getTargetPosition() - (int) (gamepad2.left_trigger * Klift));
        }

        if (  gamepad2.right_trigger > 0.2 )
        {
            _hardware._extendo.setPosition(_hardware._extendo.getTargetPosition() + (int) (gamepad2.right_trigger * Klift));
        }

        telemetry.addData("extendo", _hardware._extendo.getCurrentPosition());
        //differential intake
//        if (  gamepad2.right_trigger > 0.2 )
//        {
//            _hardware.differentialIntake.setPosition(_hardware.differentialIntake.getLinearAngle() + 0.005, _hardware.differentialIntake.getRotationAngle());
//        }
//
//        if (  gamepad2.left_trigger > 0.2 )
//        {
//            _hardware.differentialIntake.setPosition(_hardware.differentialIntake.getLinearAngle() -0.005, _hardware.differentialIntake.getRotationAngle());
//        }


        if (  gamepad2.right_stick_y > 0.2 )
        {
            //clawAngle += 0.1;
            _hardware.differentialClaw.setPosition(_hardware.differentialClaw.getLinearAngle(), _hardware.differentialClaw.getRotationAngle() + 0.005);
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

        last_right_stick = right_stick;
    }

    @Override
    void shutdown() {

    }

    public Controller1()
    {

    }
}