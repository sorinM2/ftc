package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.arcrobotics.ftclib.command.InstantCommand;

public class ChangePosition extends InstantCommand {
    public ChangePosition(double x, double y, double t)
    {
        super(
                ()-> {
                    MovementCommand.setNewTargetHeading(t);
                    MovementCommand.setNewTargetPoint(x, y, 1);
                }
        );
    }

    public ChangePosition(double x, double y, double t, double maxspeed)
    {
        super(
                ()-> {
                    MovementCommand.setNewTargetHeading(t);
                    MovementCommand.setNewTargetPoint(x, y, maxspeed);
                }
        );
    }

    public ChangePosition(double x, double y)
    {
        super(
                ()-> {
                    MovementCommand.setNewTargetPoint(x, y, 1);
                }
        );
    }
}
