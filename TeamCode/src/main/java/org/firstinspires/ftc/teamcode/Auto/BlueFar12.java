package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
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
import org.firstinspires.ftc.teamcode.data.EOALocalization;
import org.firstinspires.ftc.teamcode.data.EOAOffset;
import org.firstinspires.ftc.teamcode.pedroPathing.PPConstants;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
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

    public static double[] TURRET_POSITIONS = {8480,8550, 8450, 8300};


    public static double hoodPos = 0.11;


    public static double flywheel_target = 470_000;


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

        PedroComponent.follower().setStartingPose(new Pose(64, 9.5, Math.PI));


        paths = new BlueFar12Paths(PedroComponent.follower());


        telemetry.addData("flywheel vel: ", FlywheelNF.INSTANCE.flywheel.getRealVelocity());
        telemetry.addData("turret start pos: ", TurretNF.INSTANCE.turret.startPosition);


        telemetry.update();
    }




    @Override
    public void onStartButtonPressed() {


        telemetry.clearAll();


        //setup
        FlywheelNF.INSTANCE.setVelCatch(flywheel_target, 570_000, 2000);
        IntakeNF.INSTANCE.intake.setPower(Constants.INTAKE_POWER);
        HoodNF.INSTANCE.hood.setPosition(hoodPos);
        TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0] - TurretNF.INSTANCE.turret.startPosition);


        auto().schedule();




    }


    @Override
    public void onUpdate() {
        telemetry.addData("flywheel target vel: ", FlywheelNF.INSTANCE.flywheel.getTargetVelocity());
        telemetry.addData("flywheel current vel: ", FlywheelNF.INSTANCE.flywheel.getRealVelocity());
        telemetry.addData("turret Current: ", TurretNF.INSTANCE.turret.getCurrentPosition());
        telemetry.addData("turret error: ", TurretNF.INSTANCE.turret.getRawPositionError());
        telemetry.addData("turret target pos: ", TurretNF.INSTANCE.turret.getTargetPosition());


        telemetry.update();


    }


    @Override
    public void onStop() {

        EOAOffset offset = Constants.EOA_OFFSETS.get("auto12");

        //EOALocalization.write();
        EOALocalization.write(
                EOALocalization.autoFormatToTeleOpFormat(
                        PedroComponent.follower().getPose(),
                        offset.getXOffset(),
                        offset.getYOffset()
                ),
                TurretNF.INSTANCE.turret.startPosition
        );
    }




    private Command auto() {


        return new SequentialGroup(
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0] - TurretNF.INSTANCE.turret.startPosition),


                //PRELOAD SHOOTING
                //new FollowPath(paths.preload),
                new WaitUntil(() -> FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= flywheel_target - 2000),
                shootBalls(
                        new double[] {0.43, 0.43, 0.43},
                        new double[] {1, 1},
                        3000
                ),


                //FIRST INTAKE
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[1]- TurretNF.INSTANCE.turret.startPosition),


                RobotNF.robot.intakeClearingSpecial(1),
                new FollowPath(paths.FirstIntake),


                //FIRST RETURN
                followCancelable(paths.FirstReturn, 3000),//new FollowPath(paths.intake),
                shootBalls(
                        new double[] {0.43, 0.43, 0.43},
                        new double[] {1, 1},
                        3000
                ),

                RobotNF.robot.intakeClearingSpecial(0.25),

                //SECOND INTAKE
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[2]- TurretNF.INSTANCE.turret.startPosition),
                followCancelable(paths.SecondIntake, 4000),//new FollowPath(paths.intake),


                //SECOND RETURN

                followCancelable(paths.SecondReturn, 3500),
                shootBalls(
                        new double[] {0.43, 0.43, 0.43},
                        new double[] {1, 1},
                        3000
                ),


                //EXTRA INTAKE


                RobotNF.robot.intakeClearingSpecial(0.5),


                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[3]- TurretNF.INSTANCE.turret.startPosition),


                RobotNF.robot.intakeClearingSpecial(0.5),


                followCancelable(paths.setupForFirstIntake, 2300),
                followCancelable(paths.intakeExtra, 1300),

                //INTAKE EXTRA RETURN


                new FollowPath(paths.firstReturnn, true),

                //followCancelable(paths.firstReturnn, 9000),

                shootBalls(
                        new double[] {0.43, 0.43, 0.43},
                        new double[] {1, 1},
                        3000
                ),

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

    private Command shootBalls(double[] transferTime, double[] timeBetweenTransfers, double flywheelVelMargin) {

        ElapsedTime timer = new ElapsedTime();

        return new SequentialGroup(

                //1
                TransferNF.INSTANCE.transfer(),
                new Delay(transferTime[0]),
                TransferNF.INSTANCE.idle(),

                new WaitUntil(() -> (FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= flywheel_target - flywheelVelMargin || timer.seconds() > timeBetweenTransfers[0])),

                //2
                TransferNF.INSTANCE.transfer(),
                new Delay(transferTime[1]),
                TransferNF.INSTANCE.idle(),

                new WaitUntil(() -> (FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= flywheel_target - flywheelVelMargin || timer.seconds() > timeBetweenTransfers[1])),

                //3
                TransferNF.INSTANCE.transfer(),
                new Delay(transferTime[2]),
                TransferNF.INSTANCE.anti()
        );
    }

}


