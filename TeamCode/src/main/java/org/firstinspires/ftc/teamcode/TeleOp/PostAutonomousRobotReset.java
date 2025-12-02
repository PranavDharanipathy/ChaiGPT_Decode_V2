package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

public class PostAutonomousRobotReset extends RobotResetter {

    private ElapsedTime timer = new ElapsedTime();

    public PostAutonomousRobotReset(TeleOpBaseOpMode opMode) {

        timer.reset();

        opMode.turret.setPosition(5000);

        while (timer.milliseconds() >= 150);

        opMode.turret.setPosition(-5000);

        while (timer.milliseconds() >= 150);

        opMode.turret.setPosition(0);
    }
}
