package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.ShooterSystems.Goal;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
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

    //normalized
    public static double MIN_TURRET_POSITION_IN_DEGREES = -130, MAX_TURRET_POSITION_IN_DEGREES = 130;

    public static double TICKS_PER_DEGREE = 73.5179487179; //it should include the turret gear ratio -> (encoder rotations per turret rotation) * (8192 / 360)

    public static double[] REZERO_POSE = {0,0,0};

    private CustomMecanumDrive customDrive;

    private TurretBase turret;

    private Rev9AxisImu rev9AxisImu;

    private RobotCentricDrive robotCentricDrive = new RobotCentricDrive();

    private BetterGamepad controller1;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        controller1 = new BetterGamepad(gamepad1);

        rev9AxisImu = hardwareMap.get(Rev9AxisImu.class, Constants.MapSetterConstants.rev9AxisIMUDeviceName);
        rev9AxisImu.initialize(Constants.IMUConstants.getRev9AxisIMUParams());

        DcMotor left_front = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.leftFrontMotorDeviceName);
        DcMotor right_front = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.rightFrontMotorDeviceName);
        DcMotor left_back = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.leftBackMotorDeviceName);
        DcMotor right_back = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.rightBackMotorDeviceName);

        left_back.setDirection(DcMotor.Direction.REVERSE);

        robotCentricDrive.provideComponents(left_front, right_front, left_back, right_back, controller1);

        customDrive = new CustomMecanumDrive(hardwareMap, new Pose2d(0,0,0));

        turret = new TurretBase(hardwareMap);
        turret.setPIDFCoefficients(
                Constants.TURRET_PIDFS_COEFFICIENTS[0],
                Constants.TURRET_PIDFS_COEFFICIENTS[1],
                Constants.TURRET_PIDFS_COEFFICIENTS[2],
                Constants.TURRET_PIDFS_COEFFICIENTS[3],
                Constants.TURRET_PIDFS_COEFFICIENTS[4],
                Constants.TURRET_PIDFS_COEFFICIENTS[5],
                Constants.TURRET_PIDFS_COEFFICIENTS[6],
                Constants.TURRET_PIDFS_COEFFICIENTS[7],
                Constants.TURRET_PIDFS_COEFFICIENTS[8]
        );
        turret.reverse();
        turret.setIConstraints(Constants.TURRET_MIN_INTEGRAL_LIMIT, Constants.TURRET_MAX_INTEGRAL_LIMIT);

    }



    @Override
    public void start() {

        turretStartPosition = turret.getCurrentPosition();
        position = 0;

        Pose2d reZeroPose = new Pose2d(

                REZERO_POSE[0],
                REZERO_POSE[1],
                REZERO_POSE[2]
        );

        customDrive.updatePoseEstimate();

        //ShooterInformation.Calculator.reZeroToNormalizedPose(customDrive.localizer.getPose().position, rev9AxisImu.getRobotYawPitchRollAngles().getYaw(), reZeroPose);
        customDrive.localizer.setPose(reZeroPose);

        rev9AxisImu.resetYaw();
    }

    private double turretStartPosition;

    private double position;

    @Override
    public void loop() {

        controller1.getInformation();

        double turretCurrentPosition = turret.getCurrentPosition();

        double robotYawRad = rev9AxisImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        PoseVelocity2d robotVelocity = customDrive.updatePoseEstimate();
        Pose2d robotPose = ShooterInformation.Calculator.getBotPose(customDrive.localizer.getPose().position, robotYawRad);
        Pose2d turretPose = ShooterInformation.Calculator.getTurretPoseFromBotPose(robotPose.position, robotYawRad, turretCurrentPosition, turretStartPosition);

        Goal.GoalCoordinate goalCoordinate;

        if (robotPose.position.x > ShooterInformation.ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER) {
            goalCoordinate = goal.GOAL_COORDINATES.getCloseCoordinate();
        }
        else {
            goalCoordinate = goal.GOAL_COORDINATES.getFarCoordinate();
        }

        double angleToGoal = Goal.getAngleToGoal(turretPose.position.x, turretPose.position.y, goalCoordinate);
        double rawtt = (angleToGoal - Math.toDegrees(robotYawRad) + ShooterInformation.ShooterConstants.TURRET_ANGULAR_OFFSET);
        double tt = route(rawtt);

        telemetry.addData("angleToGoal", angleToGoal);
        telemetry.addData("rawtt", rawtt);
        telemetry.addData("tt", tt);
        telemetry.addData("rawtt in ticks", rawtt * ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE + turretStartPosition);
        telemetry.addData("rev9axis imu heading", rev9AxisImu.getRobotYawPitchRollAngles().getYaw());

        position = tt * ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE + turretStartPosition;
        double targetPosition = MathUtil.deadband(position, turret.getTargetPosition(), ShooterInformation.ShooterConstants.TURRET_DEADBAND_TICKS);
        telemetry.addData("$turret$targetPosition", targetPosition);
        turret.setPosition(targetPosition);

        turret.update();
        robotCentricDrive.update();

        telemetry.addData("turret target position", "normalized: %.4f, rezeroed: %.4f", turret.getTargetPosition(), position);
        telemetry.addData("turret start position", turretStartPosition);

        telemetry.addData("turret current position", turretCurrentPosition);

        //telemetry.addData("robot velocity", "x vel: %.4f, y vel: %.4f, ang vel: %.4f", robotVelocity.linearVel.x, robotVelocity.linearVel.y, robotVelocity.angVel);
        telemetry.addData("robot pose", "x: %.4f, y: %.4f, heading: %.4f", robotPose.position.x, robotPose.position.y, Math.toDegrees(robotYawRad));
        telemetry.addData("turret pose", "x: %.4f, y: %.4f, heading: %.4f", turretPose.position.x, turretPose.position.y, turretPose.heading.toDouble());

        telemetry.addData("turret angle to goal", angleToGoal);

        //telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());
        telemetry.update();
    }

    private double route(double rawtt) {

        if (rawtt >= MIN_TURRET_POSITION_IN_DEGREES && rawtt <= MAX_TURRET_POSITION_IN_DEGREES) return rawtt; //no need to reroute

        double[] reroutes = {rawtt - 360, rawtt + 360};
        if (reroutes[0] >= MIN_TURRET_POSITION_IN_DEGREES && reroutes[0] <= MAX_TURRET_POSITION_IN_DEGREES) {
            return reroutes[0];
        }
        else if (reroutes[1] >= MIN_TURRET_POSITION_IN_DEGREES && reroutes[1] <= MAX_TURRET_POSITION_IN_DEGREES) {
            return reroutes[1];
        }

        // go to closest limit if target position is outside the min and max
        else if (rawtt < MIN_TURRET_POSITION_IN_DEGREES) return MIN_TURRET_POSITION_IN_DEGREES;
        else return MAX_TURRET_POSITION_IN_DEGREES;
    }

}
