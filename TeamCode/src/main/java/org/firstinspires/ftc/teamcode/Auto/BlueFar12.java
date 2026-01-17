package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.FlywheelNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.HoodNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.IntakeNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.TransferNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.TurretNF;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PPConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "AUTO BLUE FAR 12", group = "Autonomous")
@Config
public class BlueFar12 extends NextFTCOpMode {

    private Telemetry telemetry;
    public Follower follower; // Pedro Pathing follower instance

    public static double[] TURRET_POSITIONS = {8450, 8550, 8650, 8350, 8600, 7000};


    private BlueFar12Paths paths;

    public BlueFar12() {
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


    public void onInit() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());


        follower = PPConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(64, 9.5, Math.PI));

        paths = new BlueFar12Paths(PedroComponent.follower());
    }


    @Override
    public void onStartButtonPressed() {

        //setup
        FlywheelNF.INSTANCE.flywheel.setVelocity(451_000, true);
        IntakeNF.INSTANCE.intake.setPower(Constants.INTAKE_POWER);
        HoodNF.INSTANCE.hood.setPosition(0.32);
        TurretNF.INSTANCE.turret.setPosition(TURRET_POSITIONS[0]);

        auto().schedule();

    }

    @Override
    public void onUpdate() {
        telemetry.addData("flywheel vel: ", FlywheelNF.INSTANCE.flywheel.getRealVelocity());

        telemetry.update();
    }


    private Command auto() {

        return new SequentialGroup(

                //shooting preloads(Turret position is already set)




                //new FollowPath(paths.preload, true),
                new WaitUntil(() -> FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= FlywheelNF.INSTANCE.flywheel.getTargetVelocity() - 100),
                //preload shooting

                RobotNF.robot.shootBalls(0.3,0.3),

                //intaking balls already set at the the human player zone
                //TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[1]),

                new FollowPath(paths.FirstIntake),


                new FollowPath(paths.FirstReturn),
                followCancelable(paths.FirstReturn, 7000),

                //intaking balls at the human  followCancelable(paths.FirstIntake, 7000), //new FollowPath(paths.firstInplayer zone
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[1]),
                followCancelable(paths.FirstReturn, 6000),//new FollowPath(paths.intake),
                new FollowPath(paths.FirstReturn, true),
                //shooting balls
                RobotNF.robot.shootBalls(0.3,0.3, 1, paths.FirstReturn),

                //intaking balls at the human player zone
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[2]),
                followCancelable(paths.SecondIntake, 6000),//new FollowPath(paths.intake),
                new FollowPath(paths.SecondIntake, false),

                followCancelable(paths.SecondReturn, 6000),
                new FollowPath(paths.SecondReturn, true),

                //second intake shooting balls
                RobotNF.robot.shootBalls(0.3,0.3, 1, paths.SecondIntake),


                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[3]),
                followCancelable(paths.IntakeExtra, 6000),
                new FollowPath(paths.IntakeExtra),

                //TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[5]),
                followCancelable(paths.ThirdReturn, 6000),
                new FollowPath(paths.ThirdReturn),

                RobotNF.robot.shootBalls(0.3, 0.3, 1, paths.ThirdReturn)
        );
    }

    boolean brokeFollowing;


    private Command followCancelable(PathChain pathChain, double millisTilCancel) {

        brokeFollowing = false;

        return new SequentialGroup(

                new InstantCommand(() -> PedroComponent.follower().followPath(pathChain)),
                new Command() {

                    private boolean firstTick = true;
                    private double startTime;
                    @Override
                    public boolean isDone() {

                        if (firstTick) {

                            startTime = System.currentTimeMillis();
                            firstTick = false;
                        }

                        return PedroComponent.follower().atParametricEnd() || System.currentTimeMillis() >= millisTilCancel + startTime;
                    }
                },
                new InstantCommand(() -> {
                    brokeFollowing = true;
                    PedroComponent.follower().breakFollowing();
                })
        );
    }

}

