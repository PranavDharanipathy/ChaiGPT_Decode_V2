package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.FlywheelNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.HoodNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.IntakeNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.TransferNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.TurretNF;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.pedroPathing.PPConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Config
@Autonomous (name = "Sling Auto BLUE FAR", group = "AAAA_MatchPurpose", preselectTeleOp = "V2TeleOp_BLUE")
public class SlingAuto_CYCLE_BLUE_FAR extends NextFTCOpMode {

    public static double[] TURRET_POSITIONS = {8150, 8300, 8300, 8300, 8400};

    private Telemetry telemetry;

    private SlingAutoPaths_CYCLE_BLUE_FAR paths;

    public SlingAuto_CYCLE_BLUE_FAR() {
        addComponents(
                new SubsystemComponent(
                        RobotNF.robot,
                        FlywheelNF.INSTANCE,
                        TurretNF.INSTANCE,
                        HoodNF.INSTANCE,
                        IntakeNF.INSTANCE,
                        TransferNF.INSTANCE
                ),
                new PedroComponent(PPConstants::createAutoFollower),
                BulkReadComponent.INSTANCE
        );
    }

    private ElapsedTime universalTimer = new ElapsedTime();

    @Override
    public void onInit() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        paths = new SlingAutoPaths_CYCLE_BLUE_FAR(PedroComponent.follower());

        PedroComponent.follower().setStartingPose(paths.startPose);
    }

    @Override
    public void onStartButtonPressed() {

        //setup
        FlywheelNF.INSTANCE.flywheel.setVelocity(456_000, true);
        IntakeNF.INSTANCE.intake.setPower(Constants.INTAKE_POWER);
        HoodNF.INSTANCE.hood.setPosition(0.16);
        TurretNF.INSTANCE.turret.setPosition(TURRET_POSITIONS[0]);

        universalTimer.reset();

        //auto().schedule();
        //auto
        new SequentialGroup(
                new ParallelRaceGroup(
                        auto(),
                        new WaitUntil(() -> universalTimer.milliseconds() > 29_000)
                ),

                TurretNF.INSTANCE.goToHomePositionCmd(),
                FlywheelNF.INSTANCE.setVel(0, true),
                TransferNF.INSTANCE.antiVeryStrong(),
                IntakeNF.INSTANCE.fullReverse(),

                new FollowPath(paths.movementRP, true)
        ).schedule();
    }

    @Override
    public void onUpdate() {

        telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());

        telemetry.addData("flywheel target velocity", FlywheelNF.INSTANCE.flywheel.getTargetVelocity());
        telemetry.addData("flywheel current velocity", FlywheelNF.INSTANCE.flywheel.getRealVelocity());
        telemetry.addData("flywheel power", FlywheelNF.INSTANCE.flywheel.getMotorPowers()[0]);

        telemetry.addData("flywheel p", FlywheelNF.INSTANCE.flywheel.p);
        telemetry.addData("flywheel i", FlywheelNF.INSTANCE.flywheel.i);
        telemetry.addData("flywheel d", FlywheelNF.INSTANCE.flywheel.d);
        telemetry.addData("flywheel v", FlywheelNF.INSTANCE.flywheel.v);

        telemetry.addData("turret target position", TurretNF.INSTANCE.turret.getTargetPosition());
        telemetry.addData("turret current position", TurretNF.INSTANCE.turret.getCurrentPosition());
        telemetry.addData("turret start position", TurretNF.INSTANCE.turret.startPosition);

        telemetry.addData("hood position", HoodNF.INSTANCE.hood.getPosition());

        telemetry.addData("intake power?", IntakeNF.INSTANCE.intake.getPower());

        telemetry.addData("brokeFollowing", brokeFollowing);
        telemetry.addData("pedro busy?", PedroComponent.follower().isBusy());

        telemetry.update();
    }

    @Override
    public void onStop() {
        RobotNF.robot.end();
    }

    private Command auto() {

        return new SequentialGroup(

                //shooting preloads
                new WaitUntil(() -> FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= FlywheelNF.INSTANCE.flywheel.getTargetVelocity() - 100),
                RobotNF.robot.shootBalls(0.4,0.3),

                //intaking balls already set at the the human player zone
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[1]),
                new FollowPath(paths.setupForFirstIntake),
                followCancelable(paths.firstIntake, 4000),//new FollowPath(paths.firstIntake),
                new FollowPath(paths.firstReturnn, true),
                //shooting balls
                RobotNF.robot.shootBalls(0.4,0.3),

                //intaking balls at the human player zone
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[2]),
                IntakeNF.INSTANCE.reverse(),
                new ParallelGroup(
                        followCancelable(paths.curvedIntake2, 4000), //new FollowPath(paths.intake),
                        RobotNF.robot.intakeClearingSpecial(1.5)
                ),
                new FollowPath(paths.curvedReturn2, true),
                //shooting balls
                RobotNF.robot.shootBalls(0.4,0.3),

                //intaking balls at the human player zone
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[3]),
                new ParallelGroup(
                        followCancelable(paths.curvedIntake2, 4000),//new FollowPath(paths.intake),
                        RobotNF.robot.intakeClearingSpecial(1.5)
                ),
                new FollowPath(paths.curvedReturn2, true),
                //shooting balls
                RobotNF.robot.shootBalls(0.4,0.3),

                //intaking balls at the human player zone
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[4]),
                new ParallelGroup(
                        followCancelable(paths.normalIntake, 4000),//new FollowPath(paths.intake),
                        RobotNF.robot.intakeClearingSpecial(1.5)
                ),
                new FollowPath(paths.normalReturn, true),
                //shooting balls
                RobotNF.robot.shootBalls(0.4,0.3)
        );
    }

    private boolean brokeFollowing = false;

    private Command followCancelable(PathChain pathChain, double millisTilCancel) {

        brokeFollowing = false;

        return new SequentialGroup(

                new InstantCommand(() -> PedroComponent.follower().followPath(pathChain)),
                new Command() {

                    private boolean firstTick = true;
                    private double startTime;

                    private boolean cancel = false;

                    @Override
                    public boolean isDone() {

                        if (firstTick) {

                            startTime = System.currentTimeMillis();
                            firstTick = false;
                        }

                        if (System.currentTimeMillis() >= millisTilCancel + startTime) {
                            cancel = true;
                            PedroComponent.follower().breakFollowing();

                            brokeFollowing = true;
                        }

                        return PedroComponent.follower().atParametricEnd() || cancel;
                    }
                }
        );
    }

}
