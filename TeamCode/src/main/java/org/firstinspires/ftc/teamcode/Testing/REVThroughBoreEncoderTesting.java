package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp (group = "testing")
public class REVThroughBoreEncoderTesting extends OpMode {

    private DcMotorEx encoder;

    public static String MOTOR_NAME = "";

    public static boolean RUN_MODE; // true = RUN_USING_ENCODER | false = RUN_WITHOUT_ENCODER

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        encoder = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        if (RUN_MODE) encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        telemetry.addData("current position", encoder.getCurrentPosition());
        telemetry.addData("velocity", encoder.getVelocity());
        telemetry.update();
    }
}
