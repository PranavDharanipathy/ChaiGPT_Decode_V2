package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.DistanceSensorTriple;

@TeleOp (group = "testing")
public class DistanceSensorTripleTesting extends OpMode {

    private DistanceSensorTriple distance3;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        distance3 = new DistanceSensorTriple(hardwareMap, Constants.MapSetterConstants.rev2mDistanceSensorNames);
    }

    @Override
    public void loop() {

        Double[] distances = distance3.getDistances();

        telemetry.addData("distances", "left: %.4f, back: %.4f, right: %.4f", distances[0], distances[1], distances[2]);
        telemetry.update();
    }
}
