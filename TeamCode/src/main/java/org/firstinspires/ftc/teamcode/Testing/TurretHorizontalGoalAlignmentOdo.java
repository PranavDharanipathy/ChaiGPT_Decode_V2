package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.ShooterSystems.Goal;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.roadrunner.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.util.MathUtil;

@Config
@TeleOp (group = "testing")
public class TurretHorizontalGoalAlignmentOdo extends OpMode {

    public enum GOAL {

        RED(Goal.GoalCoordinates.RED), BLUE(Goal.GoalCoordinates.BLUE);

        private final Goal.GoalCoordinates GOAL_COORDINATES;

        GOAL(Goal.GoalCoordinates goalCoordinates) {
            GOAL_COORDINATES = goalCoordinates;
        }
    }

    public static GOAL goal = GOAL.RED;

    public static long LOOP_TIME_DELAY = 0;

    //normalized
    public static double MIN_TURRET_POSITION_IN_DEGREES = -170, MAX_TURRET_POSITION_IN_DEGREES = 170;

    public static double TICKS_PER_DEGREE = 73.5179487179; //it should include the turret gear ratio -> (encoder rotations per turret rotation) * (8192 / 360)

    public static double[] REZERO_POSE = {0,0,0};

    private CustomMecanumDrive customDrive;

    private TurretBase turret;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        customDrive = new CustomMecanumDrive(hardwareMap, new Pose2d(0,0,0));

        turret = new TurretBase(hardwareMap);

        turret.setIConstraints(Constants.TURRET_MIN_INTEGRAL_LIMIT, Constants.TURRET_MAX_INTEGRAL_LIMIT);
        turret.setPIDFCoefficients(
                Constants.TURRET_PIDF_COEFFICIENTS[0],
                Constants.TURRET_PIDF_COEFFICIENTS[1],
                Constants.TURRET_PIDF_COEFFICIENTS[2],
                Constants.TURRET_PIDF_COEFFICIENTS[3],
                Constants.TURRET_PIDF_COEFFICIENTS[4]
        );

    }



    @Override
    public void start() {

        startPosition = turret.getCurrentPosition();
        position = 0;

        Pose2d reZeroPose = new Pose2d(

                REZERO_POSE[0],
                REZERO_POSE[1],
                REZERO_POSE[2]
        );

        customDrive.updatePoseEstimate();

        ShooterInformation.Calculator.calculateBotPoseReZeroingOffsets(customDrive.localizer.getPose(), reZeroPose);
    }

    private double startPosition;

    private double position;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void loop() {

        double MIN_TURRET_POSITION = (MIN_TURRET_POSITION_IN_DEGREES * TICKS_PER_DEGREE) + startPosition;
        double MAX_TURRET_POSITION = (MAX_TURRET_POSITION_IN_DEGREES * TICKS_PER_DEGREE) + startPosition;

        double currentPosition = turret.getCurrentPosition();

        PoseVelocity2d robotVelocity = customDrive.updatePoseEstimate();
        Pose2d robotPose = ShooterInformation.Calculator.getBotPose(customDrive.localizer.getPose());
        Pose2d turretPose = ShooterInformation.Calculator.getTurretPoseFromBotPose(robotPose, currentPosition + startPosition);

        double angleToGoal = Goal.getAngleToGoal(turretPose.position.x, turretPose.position.y, goal.GOAL_COORDINATES);
        position = Math.toDegrees(robotPose.heading.toDouble()) + (angleToGoal * ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE);

        double normalizedTargetPosition = position + startPosition;
        turret.setPosition(MathUtil.clamp(normalizedTargetPosition, MIN_TURRET_POSITION, MAX_TURRET_POSITION));

        if (timer.milliseconds() >= LOOP_TIME_DELAY) {
            turret.update();
            timer.reset();
        }

        telemetry.addData("turret target position", "normalized: %.4f, rezeroed: %.4f", normalizedTargetPosition, position);
        telemetry.addData("turret start position", startPosition);

        telemetry.addData("turret position error", turret.getPositionError());

        telemetry.addData("turret current position", currentPosition);

        telemetry.addData("robot velocity", "x vel: %.4f, y vel: %.4f, ang vel: %.4f", robotVelocity.linearVel.x, robotVelocity.linearVel.y, robotVelocity.angVel);
        telemetry.addData("robot pose", "x: %.4f, y: %.4f, heading: %.4f", robotPose.position.x, robotPose.position.y, robotPose.heading);
        telemetry.addData("turret pose", "x: %.4f, y: %.4f, heading: %.4f", turretPose.position.x, turretPose.position.y, turretPose.heading);

        telemetry.addData("turret distance from goal", Goal.getDistanceFromGoal(turretPose.position.x, turretPose.position.y, goal.GOAL_COORDINATES));
        telemetry.addData("turret angle to goal", angleToGoal);

        telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());
        telemetry.update();
    }

}
