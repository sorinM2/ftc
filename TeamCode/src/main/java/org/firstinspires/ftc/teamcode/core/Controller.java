package org.firstinspires.ftc.teamcode.core;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

public abstract class Controller {
    abstract void initialize();
    abstract void update();
    abstract void shutdown();
    protected GamepadEx gamepad;
    protected Button startButton;
}
