package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.data.EOALocalization;

@Config
@TeleOp (group = "tuning")
public class EOALocalizationOffsetTuner extends TeleOpBaseOpMode {

    public enum CurrentLocalization {
        AUTO, TELEOP
    }

    public static CurrentLocalization localizationType = CurrentLocalization.AUTO;

    //heading entered as degrees
    public static double[] AUTO_POSE = {64, 9.5, 180}; //AUTO STARTING POSE (auto format)

    public static double[] TELEOP_POSE = {-62.5, 0, 180}; //RELOCALIZATION (teleop format)

    //offsets the teleop pose to deal with the auto and teleop pose inconsistency (software to reality)
    public static double X_OFFSET;
    public static double Y_OFFSET;

    private Telemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeDevices();

        applyComponentTraits();

        waitForStart();

        while (opModeIsActive()) {

            Pose autoPose = new Pose(AUTO_POSE[0], AUTO_POSE[1], Math.toRadians(AUTO_POSE[2]));
            Pose teleOpPose = new Pose(TELEOP_POSE[0], TELEOP_POSE[1], Math.toRadians(TELEOP_POSE[2]));

            controller1.getInformation();

            if (controller1.main_buttonHasJustBeenPressed) {

                Pose pose = localizationType == CurrentLocalization.AUTO ? autoPose : teleOpPose;
                follower.setPose(pose);
            }

            follower.update();

            telemetry.addLine("main button for setting pose (set up position before clicking)");

            Pose currentPose = follower.getPose();

            if (localizationType == CurrentLocalization.TELEOP) {

                telemetry.addData("pose in current format (teleop)", currentPose);
            }
            else { //is auto

                telemetry.addData("pose in current format (auto)", currentPose);
                telemetry.addData("pose in teleop format", EOALocalization.autoFormatToTeleOpFormat(currentPose, X_OFFSET, Y_OFFSET).toString());
            }

            telemetry.update();
        }
    }
}
