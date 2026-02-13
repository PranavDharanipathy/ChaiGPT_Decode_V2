package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
import org.firstinspires.ftc.teamcode.data.LocalizationData;
import org.firstinspires.ftc.teamcode.pedroPathing.PPConstants;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
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


@Autonomous(name = "AUTO BLUE FAR 12", group = "AAA_MatchPurpose", preselectTeleOp = "V2TeleOp_BLUE")
@Config
public class BlueFar12 extends NextFTCOpMode {


    private Telemetry telemetry;

    public static double[] TURRET_POSITIONS = {8730, 8850, 8800, 8500};


    public static double hoodPos = 0.11;


    public static double flywheel_target = 451_800;


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



    private ElapsedTime universalTimer = new ElapsedTime();

    @Override
    public void onStartButtonPressed() {


        telemetry.clearAll();

        PedroComponent.follower().setMaxPower(2);


        //setup
        FlywheelNF.INSTANCE.setVelCatch(flywheel_target, 550_000, 60_000);
        IntakeNF.INSTANCE.intake.setPower(Constants.INTAKE_POWER);
        HoodNF.INSTANCE.hood.setPosition(hoodPos);
        TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0]);

        universalTimer.reset();

        //auto
        new SequentialGroup(
                new ParallelRaceGroup(
                        auto(),
                        new WaitUntil(() -> universalTimer.milliseconds() > 35_000)
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

        EOALocalization.write(
                EOALocalization.autoFormatToTeleOpFormat(
                        PedroComponent.follower().getPose(),
                        offset.getXOffset(),
                        offset.getYOffset()
                ),
                TurretNF.INSTANCE.turret.startPosition
        );
//        blackboard.put("EOALocalization", new LocalizationData(
//                EOALocalization.autoFormatToTeleOpFormat(
//                        PedroComponent.follower().getPose(),
//                        offset.getXOffset(),
//                        offset.getYOffset()
//                ),
//                TurretNF.INSTANCE.turret.startPosition
//                )
//        );
    }

    private Command auto() {


        return new SequentialGroup(
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0]),


                //PRELOAD SHOOTING
                //new FollowPath(paths.preload),
                new WaitUntil(() -> (
                        FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= flywheel_target - 1000)
                        && Math.abs(TurretNF.INSTANCE.turret.getRawPositionError()) < 200
                ),
                shootBalls(
                        new double[] {0.35, 0.375, 0.4},
                        new double[] {0.4, 0.4},
                        new double[] {0.95, 0.95},
                        300,
                        300
                ),

                //FIRST INTAKE
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[1]),


                new ParallelGroup(
                        RobotNF.robot.intakeClearingSpecial(0.4),
                        gating( //followCancelable(paths.FirstIntake, 5000),
                                paths.FirstIntake, 5000,
                                4000,
                                200, 1)
                ),


                //FIRST RETURN
                followCancelable(paths.FirstReturn, 3000),//new FollowPath(paths.intake),
                shootBalls(
                        new double[] {0.35, 0.375, 0.4},
                        new double[] {0.4, 0.4},
                        new double[] {0.95, 0.95},
                        300,
                        240
                ),

                //SECOND INTAKE
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[2]),
                new ParallelGroup(
                        RobotNF.robot.intakeClearingSpecial(0.25),
                        followCancelable(paths.SecondIntake, 4000) //new FollowPath(paths.intake),
                ),

                //SECOND RETURN

                followCancelable(paths.SecondReturn, 3500),
                shootBalls(
                        new double[] {0.35, 0.375, 0.4},
                        new double[] {0.4, 0.4},
                        new double[] {0.95, 0.95},
                        300,
                        300
                ),


                //EXTRA INTAKE

                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[3]- TurretNF.INSTANCE.turret.startPosition),

                new ParallelGroup(
                        RobotNF.robot.intakeClearingSpecial(0.5),
                        followCancelable(paths.setupForFirstIntake, 2000)
                ),
                followCancelable(paths.intakeExtra, 1000),

                //INTAKE EXTRA RETURN


                new FollowPath(paths.firstReturnn, true),

                //followCancelable(paths.firstReturnn, 9000),

                shootBalls(
                        new double[] {0.35, 0.375, 0.4},
                        new double[] {0.4, 0.4},
                        new double[] {0.95, 0.95},
                        300,
                        300
                ),

                followCancelable(paths.hpIntake, 1000),

                new FollowPath(paths.hpReturn),

                shootBalls(
                        new double[] {0.35, 0.375, 0.4},
                        new double[] {0.4, 0.4},
                        new double[] {0.95, 0.95},
                        300,
                        320
                ),



                //SET TURRET TO END POS
                TurretNF.INSTANCE.setPosition(TurretNF.INSTANCE.turret.startPosition),
                IntakeNF.INSTANCE.reverse()

        );
    }


    // compensate paths fo rstart pos
    //makes sure it shoots  3 balls





    private Command followCancelable(PathChain pathChain, double timeTilCancel) {

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

                        if (System.currentTimeMillis() >= timeTilCancel + startTime) {
                            cancel = true;
                            PedroComponent.follower().breakFollowing();

                        }

                        return PedroComponent.follower().atParametricEnd() || cancel;
                    }
                }
        );
    }

    private Command shootBalls(double[] transferTime, double[] minTimeBetweenTransfers, double[] maxTimeBetweenTransfers, double flywheelVelMargin, double secondShootFlywheelMargin) {

        ElapsedTime timer = new ElapsedTime();

        return new SequentialGroup(

                //1
                TransferNF.INSTANCE.transfer(),
                new Delay(transferTime[0]),
                TransferNF.INSTANCE.antiStrong(),

                new Delay(minTimeBetweenTransfers[0]),
                new InstantCommand((timer::reset)),
                new WaitUntil(() -> (FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= flywheel_target - secondShootFlywheelMargin || timer.seconds() > maxTimeBetweenTransfers[0])),

                //2
                TransferNF.INSTANCE.transfer(),
                new Delay(transferTime[1]),
                TransferNF.INSTANCE.antiStrong(),

                new Delay(minTimeBetweenTransfers[1]),
                new InstantCommand((timer::reset)),
                new WaitUntil(() -> (FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= flywheel_target - flywheelVelMargin || timer.seconds() > maxTimeBetweenTransfers[1])),

                //3
                TransferNF.INSTANCE.transfer(),
                new Delay(transferTime[2]),
                TransferNF.INSTANCE.antiStrong()
        );
    }

    public Command gating(PathChain cancelablePath, double timeTilCancel, double allowedTimeAtGateWhenFollowing, double uncancelledHoldTime, double cancelledHoldTime) {

        return new Command() {

            final String[] STAGES = {"START", "FOLLOW", "TIME_STOP", "HOLD_TIME_DECISION", "DELAY", "DONE"};

            String stage = STAGES[0];

            private ElapsedTime openGateTimer = new ElapsedTime();

            private double timeAlreadyAtGate = 0;

            private double holdTime = 0;

            @Override
            public boolean isDone() {

                switch (stage) {

                    case "START":

                        openGateTimer.reset();
                        PedroComponent.follower().followPath(cancelablePath);
                        stage = STAGES[1];
                        break;

                    case "FOLLOW":

                        boolean cancel = false;

                        if (openGateTimer.milliseconds() >= timeTilCancel) {

                            cancel = true;
                            PedroComponent.follower().breakFollowing();
                        }

                        if (cancel || PedroComponent.follower().atParametricEnd()) {
                            stage = STAGES[2];
                        }
                        break;

                    case "TIME_STOP":

                        timeAlreadyAtGate = openGateTimer.milliseconds();
                        stage = STAGES[3];
                        break;

                    case "HOLD_TIME_DECISION":

                        if (timeAlreadyAtGate > allowedTimeAtGateWhenFollowing) {
                            holdTime = cancelledHoldTime;
                        }
                        else {
                            holdTime = uncancelledHoldTime;
                        }

                        openGateTimer.reset();

                        stage = STAGES[4];
                        break;

                    case "DELAY":

                        if (openGateTimer.milliseconds() > holdTime) stage = STAGES[5];
                        break;
                }

                return stage.equals("DONE");
            }
        };

    }

}


