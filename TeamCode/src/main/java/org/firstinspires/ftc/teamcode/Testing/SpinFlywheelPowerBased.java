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

@Config
@TeleOp (group = "testing")
public class SpinFlywheelPowerBased extends OpMode {

    public static double POWER = 0;

    private DcMotorEx leftFlywheel, rightFlywheel;

    @Override
    public void init() {

        leftFlywheel = hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.leftFlywheelMotorDeviceName);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.rightFlywheelMotorDeviceName);

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFlywheel.setDirection(Constants.FLYWHEEL_MOTOR_DIRECTIONS[0]);
        rightFlywheel.setDirection(Constants.FLYWHEEL_MOTOR_DIRECTIONS[1]);
    }

    @Override
    public void loop() {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFlywheel.setPower(POWER);
        rightFlywheel.setPower(POWER);

        telemetry.addData("velocity", leftFlywheel.getVelocity());
        telemetry.update();
    }
}