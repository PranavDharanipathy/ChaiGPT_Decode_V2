package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.Constants;

@TeleOp (group = "testing")
public class OdoBasedDistanceFromGoalTesting extends LinearOpMode {

    private Rev9AxisImu rev9AxisImu;

    private Telemetry telemetry;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        rev9AxisImu = hardwareMap.get(Rev9AxisImu.class, Constants.MapSetterConstants.rev9AxisIMUDeviceName);
        rev9AxisImu.initialize(Constants.IMUConstants.getRev9AxisIMUParams());

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {



            AngularVelocity angularVels = rev9AxisImu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData(
                    "heading", "roll: %.4f, pitch: %.4f, yaw: %.4f",
                    angularVels.xRotationRate, angularVels.yRotationRate, angularVels.zRotationRate
            );

            telemetry.update();
        }

    }
}
