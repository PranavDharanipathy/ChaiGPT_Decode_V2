package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;

@TeleOp (group = "testing")
public class MultiIMUTesting extends LinearOpMode {

    private IMU internalIMU;
    private Rev9AxisImu rev9AxisIMU;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        internalIMU = hardwareMap.get(IMU.class, Constants.MapSetterConstants.internalIMUDeviceName);
        internalIMU.initialize(Constants.IMUConstants.getInternalIMUParams());

        rev9AxisIMU = hardwareMap.get(Rev9AxisImu.class, Constants.MapSetterConstants.rev9AxisIMUDeviceName);
        rev9AxisIMU.initialize(Constants.IMUConstants.getRev9AxisIMUParams());

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {

            AngularVelocity[] angularVelocities = {rev9AxisIMU.getRobotAngularVelocity(AngleUnit.DEGREES), internalIMU.getRobotAngularVelocity(AngleUnit.DEGREES)};
            YawPitchRollAngles[] angles = {rev9AxisIMU.getRobotYawPitchRollAngles(), internalIMU.getRobotYawPitchRollAngles()};

            // index 0 of angularVelocities and angles are Rev9AxisImu statistics while index 1 is of the internal IMU

            telemetry.addLine("REV 9 Axis IMU");
            telemetry.addData("Roll", angles[0].getRoll());
            telemetry.addData("Roll Rate", angularVelocities[0].xRotationRate);
            telemetry.addData("Pitch", angles[0].getPitch());
            telemetry.addData("Pitch Rate", angularVelocities[0].yRotationRate);
            telemetry.addData("Yaw", angles[0].getYaw());
            telemetry.addData("Yaw Rate", angularVelocities[0].zRotationRate);

            telemetry.addLine("Internal IMU");
            telemetry.addData("Roll", angles[1].getRoll());
            telemetry.addData("Roll Rate", angularVelocities[1].xRotationRate);
            telemetry.addData("Pitch", angles[1].getPitch());
            telemetry.addData("Pitch Rate", angularVelocities[1].yRotationRate);
            telemetry.addData("Yaw", angles[1].getYaw());
            telemetry.addData("Yaw Rate", angularVelocities[1].zRotationRate);
            telemetry.update();
        }

    }
}
