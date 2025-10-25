package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;

@Config
@TeleOp (group = "testing")
public class BeambreakTesting extends OpMode {

    private AdafruitBeambreakSensor beambreak;

    public static String BEAMBREAK_RECEIVER_NAME = "";
    public static String BEAMBREAK_POWER_NAME = "";

    private Telemetry telemetry;

    @Override
    public void init() {

        beambreak = new AdafruitBeambreakSensor(hardwareMap, BEAMBREAK_POWER_NAME, BEAMBREAK_RECEIVER_NAME);

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        telemetry.addData("Beam state", beambreak.isBeamBroken());
        telemetry.update();
    }
}
