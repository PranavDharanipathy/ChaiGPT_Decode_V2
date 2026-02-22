package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.data.EOALocalization;

@Config
@TeleOp(group = "testing")
public class WriteEOALocalization extends LinearOpMode {

    public static double X = 0;
    public static double Y = 0;
    public static double HEADING = 0;
    public static double TURRET_START_POSITION = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        EOALocalization.write(X, Y, HEADING, TURRET_START_POSITION);
    }
}
