package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.ShooterSystems.Goal;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.TeleOp.PostAutonomousRobotReset;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.util.Rev9AxisImuWrapped;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

@Config
@TeleOp (group = "tuning")
public class HoodRegressionTuner extends TeleOpBaseOpMode {

    public static double TRANSFER_VELOCITY = 1200;
    public static double INTAKE_POWER = 1;
    public static double FLYWHEEL_VELOCITY = 370_000;
    public static double HOOD_POSITION = 0.3;

    public enum GOAL {

        RED(Goal.GoalCoordinatesForDistance.RED.getCoordinate()), BLUE(Goal.GoalCoordinatesForDistance.BLUE.getCoordinate());

        private Goal.GoalCoordinate coord;

        GOAL(Goal.GoalCoordinate goalCoordinate) {
            coord = goalCoordinate;
        }

        public Goal.GoalCoordinate getCoordinate() {
            return coord;
        }
    }

    public static GOAL goal = GOAL.BLUE;

    private Rev9AxisImuWrapped rev9AxisImuWrapped;

    private final RobotCentricDrive robotCentricDrive = new RobotCentricDrive();

    @Override
    public void runOpMode() {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeDevices();
        rev9AxisImuWrapped = new Rev9AxisImuWrapped(rev9AxisImu);

        applyComponentTraits();

        robotCentricDrive.provideComponents(left_front, right_front, left_back, right_back, controller1);

        //setup lynx module
        setUpLynxModule();

        //telemetry.speak("SIX SEVEN");

        if (isStopRequested()) return;
        waitForStart();

        //run robot reset
        RobotResetter robotReset = new PostAutonomousRobotReset(this);

        while (opModeIsActive() && !isStopRequested()) {

            // clear data at start of loop
            clearCacheOfLynxModule();

            controller2.getInformation();

            hoodAngler.setPosition(HOOD_POSITION);
            intake.setPower(INTAKE_POWER);
            transfer.setVelocity(TRANSFER_VELOCITY);
            flywheel.setVelocity(FLYWHEEL_VELOCITY, true);
            flywheel.updateKvBasedOnVoltage();
            flywheel.update();

            if (controller2.main_buttonHasJustBeenPressed) {
                relocalization(ShooterInformation.Odometry.RELOCALIZATION_POSES.BACK);
            }

            follower.update();

            double robotYawRad = rev9AxisImuWrapped.getYaw(AngleUnit.RADIANS);
            Pose robotPose = follower.getPose();
            Pose turretPose = ShooterInformation.Calculator.getTurretPoseFromBotPose(robotPose, 0, 0);

            double distanceToGoal = Goal.getDistanceFromGoal(turretPose.getX(), turretPose.getY(), goal.getCoordinate());

            robotCentricDrive.update();

            telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());
            telemetry.addData("(Predicted) Run speed percentage", "%.2f", TickrateChecker.getRunSpeedPercentage());

            telemetry.addData("p", flywheel.p);
            telemetry.addData("i", flywheel.i);
            telemetry.addData("d", flywheel.d);
            telemetry.addData("v", flywheel.v);

            telemetry.addData("flywheel current velocity", flywheel.getRealVelocity());
            telemetry.addData("flywheel target velocity", flywheel.getTargetVelocity());

            telemetry.addData("turret current position", turret.getCurrentPosition());
            telemetry.addData("turret target position", turret.getTargetPosition());
            telemetry.addData("turret position error", turret.getPositionError());

            telemetry.addData("turret position + robot heading", "%.2f, %.2f, %.2f", turretPose.getX(), turretPose.getY(), Math.toDegrees(robotYawRad));
            telemetry.addData("distance to goal", distanceToGoal);

            telemetry.addData("goal coordinate", "x:%.2f, y:%.2f", goal.getCoordinate().getX(), goal.getCoordinate().getY());

            telemetry.update();

        }

        //end
        closeLynxModule();

    }

    private void relocalization(ShooterInformation.Odometry.RELOCALIZATION_POSES pose) {

        double heading = ShooterInformation.Odometry.REZERO_POSES[pose.getPoseIndex()][2];

        Pose reZeroPose = new Pose(

                ShooterInformation.Odometry.REZERO_POSES[pose.getPoseIndex()][0],
                ShooterInformation.Odometry.REZERO_POSES[pose.getPoseIndex()][1],
                Math.toRadians(heading)
        );

        follower.setPose(reZeroPose);
        rev9AxisImuWrapped.setYaw(heading);
    }

}
