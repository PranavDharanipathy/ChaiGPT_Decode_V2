package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.ShooterSystems.Goal;
import org.firstinspires.ftc.teamcode.TeleOp.PostAutonomousRobotReset;
import org.firstinspires.ftc.teamcode.TeleOp.Shooter;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

@Config
@TeleOp(group = "tuning")
public class TurretHysteresisTuner extends TeleOpBaseOpMode {

    public static Goal.GoalCoordinates GOAL = Goal.GoalCoordinates.BLUE;

    public static double A_NORMAL = 7.2;
    public static double T_NORMAL = 1.5;
    public static double T_MAX = 2.5;

    private RobotCentricDrive robotCentricDrive = new RobotCentricDrive();
    private Shooter shooter = new Shooter();

    private Telemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeDevices();
        applyComponentTraits();

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        robotCentricDrive.provideComponents(left_front, right_front, left_back, right_back, controller1);
        shooter.provideComponents(flywheel, turret, hoodAngler, follower, rev9AxisImu, controller1, controller2);

        waitForStart();

        RobotResetter robotReset = new PostAutonomousRobotReset(this);

        shooter.start(GOAL);

        while (opModeIsActive()) {

            controller1.getInformation();

            shooter.update();

            telemetry.addData("Turret lookahead time", shooter.getTurretTimeLookahead());
            telemetry.update();
        }

    }

    private double getTurretFuturePosePredictionTime(double turretAcceleration) {

        final double NORMA_ACCEL = Math.toRadians(A_NORMAL);
        final double NORMAL = T_NORMAL;
        final double MAX = T_MAX;

        final double turretAccel = Math.abs(turretAcceleration);

        double futurePredictionTime = NORMAL * (1 + Math.sqrt(NORMA_ACCEL / (turretAccel + 1e-3)));

        return MathUtil.clamp(futurePredictionTime, NORMAL, MAX);
    }

}
