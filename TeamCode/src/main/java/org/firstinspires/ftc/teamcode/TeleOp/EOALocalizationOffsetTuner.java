package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.data.EOALocalization;
import org.firstinspires.ftc.teamcode.util.Rev9AxisImuWrapped;

@Config
@TeleOp (group = "tuning")
public class EOALocalizationOffsetTuner extends TeleOpBaseOpMode {

    enum CurrentLocalization {
        AUTO, TELEOP
    }

    public static CurrentLocalization localizationType = CurrentLocalization.AUTO;

    //heading entered as degrees
    public static double[] AUTO_POSE = {64, 9.5, 180}; //AUTO STARTING POSE (auto format)

    public static double[] TELEOP_POSE = {-62.5, 0, 180}; //RELOCALIZATION (teleop format)

    //offsets the teleop pose to deal with the auto and teleop pose inconsistency (software to reality)
    public static double X_OFFSET;
    public static double Y_OFFSET;

    private Rev9AxisImuWrapped rev9AxisImuWrapped;

    private Telemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        rev9AxisImuWrapped = new Rev9AxisImuWrapped(rev9AxisImu);

        while (opModeIsActive()) {

            Pose autoPose = new Pose(AUTO_POSE[0], AUTO_POSE[1], Math.toRadians(AUTO_POSE[2]));
            Pose teleOpPose = new Pose(TELEOP_POSE[0], TELEOP_POSE[1], Math.toRadians(TELEOP_POSE[2]));

            controller1.getInformation();

            if (controller1.dpad_upHasJustBeenPressed) {
                follower.setPose(autoPose);
            }
            else if (controller1.dpad_downHasJustBeenPressed) {

            }

            follower.update();

            telemetry.addLine("dpad up for auto mode (set up position before clicking)");
            telemetry.addLine("dpad up for teleop mode (set up position before clicking)");

            Pose currentPose = ShooterInformation.Calculator.getBotPose(follower.getPose(), rev9AxisImuWrapped.getYaw(AngleUnit.RADIANS));

            if (localizationType == CurrentLocalization.TELEOP) {

                telemetry.addData("distance to auto pose", autoPose.minus(currentPose));

                telemetry.addData("pose in current format (teleop)", currentPose);
                telemetry.addData("pose in auto format", EOALocalization.teleOpFormatToAutoFormat(currentPose, rev9AxisImuWrapped, X_OFFSET, Y_OFFSET));
            }
            else { //is auto

                telemetry.addData("distance to teleop pose", teleOpPose.minus(currentPose));

                telemetry.addData("pose in current format (auto)", currentPose);
                telemetry.addData("pose in teleop format", EOALocalization.autoFormatToTeleOpFormat(currentPose, rev9AxisImuWrapped, X_OFFSET, Y_OFFSET));
            }

            telemetry.update();
        }
    }
}
