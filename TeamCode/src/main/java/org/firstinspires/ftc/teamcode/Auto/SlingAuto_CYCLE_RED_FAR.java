package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.FlywheelNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.HoodNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.IntakeNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.TransferNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.TurretNF;
import org.firstinspires.ftc.teamcode.pedroPathing.PPConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Config
@Autonomous (name = "Sling Auto RED FAR", group = "AAAA_MatchPurpose", preselectTeleOp = "V2TeleOp_RED")
public class SlingAuto_CYCLE_RED_FAR extends NextFTCOpMode {

    public static double TURRET_POSITION = -7700;

    private Telemetry telemetry;

    private SlingAutoPaths_CYCLE_RED_FAR paths;

    public SlingAuto_CYCLE_RED_FAR() {
        addComponents(
                new SubsystemComponent(
                        RobotNF.robot,
                        FlywheelNF.INSTANCE,
                        TurretNF.INSTANCE,
                        HoodNF.INSTANCE,
                        IntakeNF.INSTANCE,
                        TransferNF.INSTANCE
                ),
                new PedroComponent(PPConstants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        paths = new SlingAutoPaths_CYCLE_RED_FAR(PedroComponent.follower());

        PedroComponent.follower().setStartingPose(paths.startPose);
    }

    @Override
    public void onStartButtonPressed() {

        auto().schedule();

        RobotNF.robot.setFlywheelVelAIR(455_000);
        RobotNF.robot.intake();
        RobotNF.robot.hoodTo(0.16);
        RobotNF.robot.turretTo(TURRET_POSITION);
    }

    @Override
    public void onUpdate() {

        telemetry.addData("flywheel target velocity", FlywheelNF.INSTANCE.flywheel.getTargetVelocity());
        telemetry.addData("flywheel current velocity", FlywheelNF.INSTANCE.flywheel.getRealVelocity());

        telemetry.addData("turret target position", TurretNF.INSTANCE.turret.getTargetPosition());
        telemetry.addData("turret current position", TurretNF.INSTANCE.turret.getCurrentPosition());

        telemetry.update();
    }

    @Override
    public void onStop() {
        RobotNF.robot.stop();
    }

    private Command auto() {

        return new SequentialGroup(

                //shooting preloads
                RobotNF.robot.waitTilFlywheelAtVel(),
                RobotNF.robot.transfer(0.4,0.2),

                //intaking balls already set at the the human player zone
                new FollowPath(paths.setupForFirstIntake),
                new FollowPath(paths.firstIntake),
                new ParallelGroup(
                        new FollowPath(paths.firstReturnn),
                        //shooting balls
                        RobotNF.robot.transfer(0.4,0.2, 2, paths.firstReturnn)
                ),

                //intaking balls at the human player zone
                new FollowPath(paths.intake),
                new ParallelGroup(
                        new FollowPath(paths.returnn),
                        //shooting balls
                        RobotNF.robot.transfer(0.4,0.2, 2, paths.firstReturnn)
                ),

                //intaking balls at the human player zone
                new FollowPath(paths.intake),
                new ParallelGroup(
                        new FollowPath(paths.returnn),
                        //shooting balls
                        RobotNF.robot.transfer(0.4,0.2, 2, paths.firstReturnn)
                )
        );
    }

}
