package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.PIPELINES;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.pedroPathing.PPConstants;
import org.firstinspires.ftc.teamcode.util.MathUtil;

@Config
@Configurable
@Autonomous (group = "testing")
public class LimelightGoalApriltagLocalization extends OpMode {

    public static boolean LL_ROBOT_CHECKBOX = true;
    public static boolean ODO_ROBOT_CHECKBOX = true;
    public static boolean TAG_BUILT_IN_CHECKBOX = true;
    public static boolean TAG_MT2_CHECKBOX = true;
    public static boolean LINE_OF_SIGHT_CHECKBOX = true;

    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private Follower follower;
    private Limelight3A limelight;

    private Telemetry telemetry;

    @Override
    public void init() {

        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());

        follower = PPConstants.createTeleOpFollower(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, Constants.MapSetterConstants.limelight3AUSBDeviceName);

        telemetry.addLine("Setup robot pointing straight!");
        telemetry.update();
    }

    @Override
    public void start() {

        telemetry.clearAll();

        follower.setHeading(0);

        limelight.setPollRateHz(ShooterInformation.CameraConstants.CAMERA_POLL_RATE);
        limelight.start();
        limelight.pipelineSwitch(PIPELINES.BLUE_PIPELINE.getPipelineIndex());
    }

    @Override
    public void loop() {

        LLStatus llStatus = limelight.getStatus();

        telemetry.addData("is connected", limelight.isConnected());
        telemetry.addData("is running", limelight.isRunning());
        telemetry.addData("fps", llStatus.getFps());
        telemetry.addData("cpu usage", llStatus.getCpu());

        follower.update();
        Pose odoPose = follower.getPose();
        limelight.updateRobotOrientation(Math.toDegrees(odoPose.getHeading()));

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {

            LLResultTypes.FiducialResult tag = llResult.getFiducialResults().get(0);

            Pose3D tagPose3D = tag.getTargetPoseRobotSpace();
            Pose3D robotPose3D = llResult.getBotpose_MT2();
            double distanceFromTag = llResult.getBotposeAvgDist();

            double xRobot = MathUtil.metersToInches(robotPose3D.getPosition().x);
            double yRobot = MathUtil.metersToInches(robotPose3D.getPosition().y);
            double yawRobot = robotPose3D.getOrientation().getYaw(AngleUnit.RADIANS);

            Pose robotPose = new Pose(xRobot, yRobot, yawRobot).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            double xTagBuiltIn = MathUtil.metersToInches(tagPose3D.getPosition().x);
            double yTagBuiltIn = MathUtil.metersToInches(tagPose3D.getPosition().y);
            double yawTagBuiltIn = tagPose3D.getOrientation().getYaw(AngleUnit.RADIANS);

            Pose tagPoseBuiltIn = new Pose(xTagBuiltIn, yTagBuiltIn, yawTagBuiltIn).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            double xTagMT2 = MathUtil.metersToInches(robotPose3D.getPosition().x + distanceFromTag * Math.cos(odoPose.getHeading()));
            double yTagMT2 = MathUtil.metersToInches(robotPose3D.getPosition().y + distanceFromTag * Math.sin(odoPose.getHeading()));
            double yawTagMT2 = odoPose.getHeading();

            Pose tagPoseMT2 = new Pose(xTagMT2, yTagMT2, yawTagMT2).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            if (LL_ROBOT_CHECKBOX) drawRobot(robotPose, new Style("", "#3F51B5", 0.75));
            if (ODO_ROBOT_CHECKBOX) drawRobot(odoPose, new Style("", "#49C061", 0.75));
            if (TAG_BUILT_IN_CHECKBOX) drawCircle(poseToPoseVector(tagPoseBuiltIn), 2, "#f06C00");
            if (TAG_MT2_CHECKBOX) drawCircle(poseToPoseVector(tagPoseMT2), 2, "#3F51B5");
            if (LINE_OF_SIGHT_CHECKBOX) drawLine(robotPose, tagPoseBuiltIn, "#00FF00");

            panelsField.update();

            telemetry.addData("Robot pose (Pedro follower)", odoPose.toString());
            telemetry.addData("Robot pose (LL)", robotPose.toString());
            telemetry.addData("Tag pose (built-in)", tagPoseBuiltIn.toString());
            telemetry.addData("Tag pose (MT2)", tagPoseMT2.toString());
            telemetry.addData("Distant to tag", distanceFromTag);
        }
        else {
            telemetry.addLine("no LL data");
        }

        telemetry.update();
    }

    public static Pose poseToPoseVector(Pose pose) {
        return new Pose(pose.getX(), pose.getY());
    }

    public static void drawCircle(Pose pose, double r, String colorHex) {
        panelsField.setStyle(new Style("", colorHex, 1));
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(r);
    }

    public static void drawLine(Pose startVector, Pose endVector, String colorHex) {
        panelsField.setStyle(new Style("", colorHex, 1));
        panelsField.moveCursor(startVector.getX(), startVector.getY());
        panelsField.line(endVector.getX(), endVector.getY());
    }

    public static void drawRobot(Pose pose, Style style) {

        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(8);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * 8);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }
}
