package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.EmbeddedControlHubModule;




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


@Autonomous(name = "AUTO BLUE FAR 12", group = "AAA_MatchPurpose")
@Config
public class BlueFar12 extends NextFTCOpMode {


    private Telemetry telemetry;
    public Follower follower;


    public static double[] TURRET_POSITIONS = {8500,8550, 8450, 8450};


    public static double hoodPos = 0.11;


    public static double flywheel_target = 465_000;


    public static Pose Blue_Auto_End_Pose = new Pose(0, 0, 0);


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
                new PedroComponent(PPConstants::createAutoFollower),
                BulkReadComponent.INSTANCE
        );
    }




    public void onInit() {


        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());




        follower = PPConstants.createAutoFollower(hardwareMap);
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
        TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0] - TurretNF.INSTANCE.turret.startPosition);


        auto().schedule();




    }


    @Override
    public void onUpdate() {
        telemetry.addData("flywheel vel: ", FlywheelNF.INSTANCE.flywheel.getRealVelocity());
        telemetry.addData("turret Current: ", TurretNF.INSTANCE.turret.getCurrentPosition());
        telemetry.addData("turret error: ", TurretNF.INSTANCE.turret.getRawPositionError());
        telemetry.addData("turret target pos: ", TurretNF.INSTANCE.turret.getTargetPosition());


        telemetry.update();


        follower.update();


    }


    @Override
    public void onStop() {


        follower.update();


        Blue_Auto_End_Pose = follower.getPose();


    }




    private Command auto() {


        return new SequentialGroup(
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0] - TurretNF.INSTANCE.turret.startPosition),


                //PRELOAD SHOOTING
                //new FollowPath(paths.preload),
                new WaitUntil(() -> FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= FlywheelNF.INSTANCE.flywheel.getTargetVelocity() - 100),
                RobotNF.robot.shootBalls(0.18,0.8),


                //FIRST INTAKE
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[1]- TurretNF.INSTANCE.turret.startPosition),


                RobotNF.robot.intakeClearingSpecial(1),
                new FollowPath(paths.FirstIntake),


                //FIRST RETURN
                followCancelable(paths.FirstReturn, 15000),//new FollowPath(paths.intake),
                RobotNF.robot.shootBalls(0.21,0.8, 3, paths.FirstReturn),

                RobotNF.robot.intakeClearingSpecial(0.25),

                //SECOND INTAKE
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[2]- TurretNF.INSTANCE.turret.startPosition),
                followCancelable(paths.SecondIntake, 8000),//new FollowPath(paths.intake),


                //SECOND RETURN

                followCancelable(paths.SecondReturn, 10000),
                RobotNF.robot.shootBalls(0.23,0.8, 3, paths.SecondReturn),


                //EXTRA INTAKE


                RobotNF.robot.intakeClearingSpecial(0.5),


                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[3]- TurretNF.INSTANCE.turret.startPosition),




                followCancelable(paths.setupForFirstIntake, 2300),
                followCancelable(paths.intakeExtra, 1300),


                RobotNF.robot.intakeClearingSpecial(0.5),


                //INTAKE EXTRA RETURN


                new FollowPath(paths.firstReturnn, true),

                //followCancelable(paths.firstReturnn, 9000),




                RobotNF.robot.shootBalls(0.23, 0.7, 1, paths.firstReturnn),


                //SET TURRET TO END POS
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



