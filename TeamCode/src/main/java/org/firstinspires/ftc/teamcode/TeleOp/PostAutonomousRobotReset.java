package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

public class PostAutonomousRobotReset extends RobotResetter {

    public PostAutonomousRobotReset(TeleOpBaseOpMode opMode) {

        opMode.turret.setPosition(opMode.turret.startPosition);
    }
}
