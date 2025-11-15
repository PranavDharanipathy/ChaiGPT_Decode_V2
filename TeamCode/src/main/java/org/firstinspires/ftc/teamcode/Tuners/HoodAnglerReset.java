package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.HoodAngler;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;

@Config
@TeleOp (group = "tuning")
public class HoodAnglerReset extends LinearOpMode {

    private HoodAngler hoodAngler;

    public static int STAGE = 0;

    private final Telemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());;

    @Override
    public void runOpMode() {

        hoodAngler = new HoodAngler(hardwareMap, Constants.MapSetterConstants.hoodAnglerLeftServoDeviceName, Constants.MapSetterConstants.hoodAnglerRightServoDeviceName);
        hoodAngler.setServoDirections(Constants.HOOD_ANGLER_SERVO_DIRECTIONS);

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {

            switch (STAGE) {

                case 0:

                    addSingleLine("desynch hood");
                    break;

                case 1:

                    hoodAngler.setPosition(0.5);
                    addSingleLine("hood angler initial movement");
                    break;


                case 2:
                    hoodAngler.setPosition(Constants.HOOD_ANGLER_INITIAL_RESETTING_POSITION);
                    addSingleLine("hood angler set to resetting position");
                    break;

                case 3:

                    addSingleLine("rest hood on hood angler");
                    break;

                case 4:

                    hoodAngler.setPosition(ShooterInformation.ShooterConstants.HOOD_ANGLER_MIN_POSITION);
                    addSingleLine("hood retracted");
                    break;
            }
        }
    }

    private void addSingleLine(String message) {

        telemetry.clearAll();
        telemetry.addLine(message);
        telemetry.update();
    }
}
