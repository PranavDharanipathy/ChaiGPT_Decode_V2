package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.ModifiedPIDFMotor;

@Config
@TeleOp(group = "tuning")
public class ModifiedPIDFMotorTuner extends OpMode {

    private ModifiedPIDFMotor motor;

    public static double kp, ki, kd, kf, KF_ENABLE_RANGE;
    public static int targetPosition;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = new ModifiedPIDFMotor(hardwareMap, "motor");
        motor.initialize(kp, ki, kd, kf, KF_ENABLE_RANGE);
    }

    @Override
    public void loop() {

        motor.setPosition(targetPosition);
        motor.updateCoefficients(kp, ki, kd, kf);
        motor.update();

        telemetry.addData("target position", motor.internalMotor.getTargetPosition());
        telemetry.addData("current position", motor.internalMotor.getCurrentPosition());
        telemetry.update();

    }
}