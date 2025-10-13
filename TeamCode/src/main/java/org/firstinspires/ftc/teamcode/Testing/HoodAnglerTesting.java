package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.TurretSystems.HoodAngler;

@Config
@TeleOp(group = "testing")
public class HoodAnglerTesting extends LinearOpMode {

    private HoodAngler hoodAngler;

    public static double ANGLE;

    @Override
    public void runOpMode() {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        hoodAngler = new HoodAngler(hardwareMap, Constants.MapSetterConstants.hoodAnglerLeftServoDeviceName, Constants.MapSetterConstants.hoodAnglerRightServoDeviceName);

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {
            hoodAngler.setAngle(ANGLE);

            telemetry.addData("Left servo position", hoodAngler.$getLeftServoPosition());
            telemetry.addData("Right servo position", hoodAngler.$getRightServoPosition());
            telemetry.update();
        }

    }
}
