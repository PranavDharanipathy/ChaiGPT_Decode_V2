package org.firstinspires.ftc.teamcode.Auton;

import org.firstinspires.ftc.teamcode.ShooterSystems.HoodAngler;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

public class ForAutonomousRobotReset extends RobotResetter {

    public ForAutonomousRobotReset(HoodAngler hoodAngler) {

        hoodAngler.setPosition(0.5);

    }
}
