package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

    public static double[] TURRET_POSITIONS = {8500,8600, 8650, 8450};

    public static double hoodPos = 0.11;

    public static double flywheel_target = 455_000;


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

        telemetry.addData("flywheel vel: ", FlywheelNF.INSTANCE.flywheel.getRealVelocity());
        telemetry.addData("turret start pos: ", TurretNF.INSTANCE.turret.startPosition);

        telemetry.update();
    }


    @Override
    public void onStartButtonPressed() {

        telemetry.clearAll();

        //setup
        FlywheelNF.INSTANCE.flywheel.setVelocity(flywheel_target, true);
        IntakeNF.INSTANCE.intake.setPower(Constants.INTAKE_POWER);
        HoodNF.INSTANCE.hood.setPosition(hoodPos);
        TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0]);

        auto().schedule();


    }

    @Override
    public void onUpdate() {
        telemetry.addData("flywheel vel: ", FlywheelNF.INSTANCE.flywheel.getRealVelocity());
        telemetry.addData("turret curr pos: ", TurretNF.INSTANCE.turret.getCurrentPosition());

        telemetry.update();

    }


    private Command auto() {

        return new SequentialGroup(
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0] - TurretNF.INSTANCE.turret.startPosition),
                //PRELOAD SHOOTING

                new WaitUntil(() -> FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= FlywheelNF.INSTANCE.flywheel.getTargetVelocity() - 100),
                //preload shooting

                RobotNF.robot.shootBalls(0.32,0.5),
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[1]- TurretNF.INSTANCE.turret.startPosition),



                //intaking balls already set at the the human player zone
                //TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[1]),

                //RobotNF.robot.intakeClearingSpecial(0.5),
                new FollowPath(paths.FirstIntake),

                //intaking balls at the human  followCancelable(paths.FirstIntake, 7000), //new FollowPath(paths.firstInplayer zone

                followCancelable(paths.FirstReturn, 8000),//new FollowPath(paths.intake),

                //shooting balls
                RobotNF.robot.shootBalls(0.32,0.6, 3, paths.FirstReturn),


                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[2]- TurretNF.INSTANCE.turret.startPosition),
                //intaking balls at the human player zone

                followCancelable(paths.SecondIntake, 8000),//new FollowPath(paths.intake),

                //RobotNF.robot.intakeClearingSpecial(0.3),

                followCancelable(paths.SecondReturn, 8000),

                //second intake shooting balls
                RobotNF.robot.shootBalls(0.32,0.6, 3, paths.SecondReturn),

               // RobotNF.robot.intakeClearingSpecial(0.3),
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[3]- TurretNF.INSTANCE.turret.startPosition),

                //intake extra balls from corner

                followCancelable(paths.setupForFirstIntake, 3000),

                followCancelable(paths.intakeExtra, 2000),

                followCancelable(paths.firstReturnn, 4000),


                RobotNF.robot.shootBalls(0.32, 0.6, 1, paths.firstReturnn),


                TurretNF.INSTANCE.setPosition(TurretNF.INSTANCE.turret.startPosition)


        );
    }

    // compensate paths fo rstart pos
    //makes sure it shoots  3 balls

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

