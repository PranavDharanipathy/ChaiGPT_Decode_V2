package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.chaigptrobotics.shenanigans.Peak;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.HoodAngler;

@Peak
@Config
@TeleOp (group = "tuning")
public class HoodAnglerPositionRegressionBuilder extends LinearOpMode {

    public static double HOOD_ANGLER_POSITION;

    public static double FLYWHEEL_VELOCITY;

    private HoodAngler hoodAngler;

    private Telemetry telemetry;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        hoodAngler = new HoodAngler(hardwareMap, Constants.MapSetterConstants.hoodAnglerLeftServoDeviceName, Constants.MapSetterConstants.hoodAnglerRightServoDeviceName);
        hoodAngler.setServoDirections(Constants.HOOD_ANGLER_SERVO_DIRECTIONS);


        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {

//            telemetry.addData("Hood angler position",hoodAngler.getPosition());
//            telemetry.addData("Current flywheel velocity", );
            telemetry.update();
        }
    }
}
