package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
@TeleOp (group = "testing")
public class SpinFlywheelPowerBased extends OpMode {

    public static double LEFT_POWER = 0;
    public static double RIGHT_POWER = 0;

    private DcMotorEx leftFlywheel, rightFlywheel;
    private Encoder encoder;

    @Override
    public void init() {

        leftFlywheel = hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.leftFlywheelMotorDeviceName);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.rightFlywheelMotorDeviceName);

        leftFlywheel.setDirection(Constants.FLYWHEEL_MOTOR_DIRECTIONS[0]);
        rightFlywheel.setDirection(Constants.FLYWHEEL_MOTOR_DIRECTIONS[1]);

        encoder = new Encoder(leftFlywheel);
        encoder.setDirection(Encoder.Direction.FORWARD);


        leftFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFlywheel.setPower(LEFT_POWER);
        rightFlywheel.setPower(RIGHT_POWER);

        telemetry.addData("encoder velocity", "corrected: %.4f, raw: %.4f, estimate: %.4f", encoder.getCorrectedVelocity(), encoder.getRawVelocity(), encoder.getVelocityEstimate());
        telemetry.addData("leftFlywheel power", leftFlywheel.getPower());
        telemetry.addData("rightFlywheel power", rightFlywheel.getPower());
        telemetry.update();
    }
}