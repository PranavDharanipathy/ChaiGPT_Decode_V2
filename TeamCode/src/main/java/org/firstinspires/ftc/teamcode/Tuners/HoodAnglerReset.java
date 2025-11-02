package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.HoodAngler;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;

@Config
@TeleOp (group = "tuning")
public class HoodAnglerReset extends LinearOpMode {

    private HoodAngler hoodAngler;

    public static int STAGE = 0;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

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

                    hoodAngler.setPosition(Constants.HOOD_ANGLER_INITIAL_RESETTING_POSITION);
                    addSingleLine("hood angler set to initial resetting position");
                    break;

                case 2:

                    addSingleLine("rest hood on hood angler");
                    break;

                case 3:

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
