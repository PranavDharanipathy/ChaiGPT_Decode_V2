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
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Autonomous(name = "AUTO BLUE FAR 15", group = "AAA_MatchPurpose", preselectTeleOp = "V2TeleOp_BLUE")
@Config
public class BlueFar15 extends NextFTCOpMode {
    private Telemetry telemetry;

    public static double[] TURRET_POSITIONS = {8750, 8750, 8800, 8900, 8750};

    //CHANGED HOOD POS FROM 0.11 to 0.19(shoots slightly higher)
    public static double hoodPos = 0.19;
    public static double flywheel_target = 415_000;

    private BlueFar15Paths paths;


    public BlueFar15() {
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


        paths = new BlueFar15Paths(PedroComponent.follower());


        telemetry.addData("flywheel vel: ", FlywheelNF.INSTANCE.flywheel.getRealVelocity());
        telemetry.addData("turret start pos: ", TurretNF.INSTANCE.turret.startPosition);


        telemetry.update();
    }

    private ElapsedTime universalTimer = new ElapsedTime();

    private ElapsedTime shootTime = new ElapsedTime();

    @Override
    public void onStartButtonPressed() {


        telemetry.clearAll();

        PedroComponent.follower().setMaxPower(2);


        //setup
        FlywheelNF.INSTANCE.setVelCatch(flywheel_target, 423_000, 300_000);
        IntakeNF.INSTANCE.intake.setPower(Constants.INTAKE_POWER);
        HoodNF.INSTANCE.hood.setPosition(hoodPos);
        TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0]);

        universalTimer.reset();

        shootTime.reset();

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
        telemetry.addLine();

        telemetry.addData("flywheel power", FlywheelNF.INSTANCE.flywheel.getPower());
        telemetry.addLine();

        telemetry.addData("turret Current: ", TurretNF.INSTANCE.turret.getCurrentPosition());
        telemetry.addData("turret error: ", TurretNF.INSTANCE.turret.getError());
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

    Command resetShootTimer() {
        return new InstantCommand(
                () -> shootTime.reset());
    }

    private Command auto() {


        return new SequentialGroup(



                //PRELOAD SHOOTING
                //new FollowPath(paths.preload),


                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(
                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0]),

                                new WaitUntil(() -> (
                                        FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= flywheel_target - 1000)

                                        //&& Math.abs(TurretNF.INSTANCE.turret.getError()) < 200
                                ),
                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0]),
                                shootBalls(
                                        new double[] {0.35, 0.375, 0.4},
                                        new double[] {0, 0},
                                        new double[] {0.4, 0.4},
                                        300,
                                        450
                                )
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),



                //FIRST INTAKE



                new ParallelGroup(
                        RobotNF.robot.intakeClearingSpecial(0.4),
                        gating( //followCancelable(paths.FirstIntake, 5000),
                                paths.FirstIntake, 5000,
                                4000,
                                200, 1)
                ),


                //FIRST RETURN
                followCancelable(paths.FirstReturn, 4000),//new FollowPath(paths.intake),

                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(
                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[1]),

                                new WaitUntil(() -> (
                                        FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= flywheel_target - 1000)
                                        //&& Math.abs(TurretNF.INSTANCE.turret.getError()) < 200
                                ),
                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[1]),
                                shootBalls(
                                        new double[] {0.35, 0.375, 0.4},
                                        new double[] {0, 0},
                                        new double[] {0.4, 0.4},
                                        300,
                                        450
                                )
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),

                //SECOND INTAKE
                new ParallelGroup(
                        RobotNF.robot.intakeClearingSpecial(0.25),
                        followCancelable(paths.SecondIntake, 4000) //new FollowPath(paths.intake),
                ),

                //SECOND RETURN

                followCancelable(paths.SecondReturn, 4500),

                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(
                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[2]),

                                new WaitUntil(() -> (
                                        FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= flywheel_target - 1000)
                                        //&& Math.abs(TurretNF.INSTANCE.turret.getError()) < 200
                                ),
                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[2]),

                                shootBalls(
                                        new double[] {0.35, 0.375, 0.4},
                                        new double[] {0, 0},
                                        new double[] {0.4, 0.4},
                                        300,
                                        450
                                )
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),


                //EXTRA INTAKE


                new ParallelGroup(
                        RobotNF.robot.intakeClearingSpecial(0.5),
                        followCancelable(paths.setupForFirstIntake, 7000)
                ),
                followCancelable(paths.hpIntake, 4000),

                //INTAKE EXTRA RETURN


                new FollowPath(paths.hpReturn, true),

                //followCancelable(paths.firstReturnn, 9000),


                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(
                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[3]- TurretNF.INSTANCE.turret.startPosition),

                                new WaitUntil(() -> (
                                        FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= flywheel_target - 1000)
                                        //&& Math.abs(TurretNF.INSTANCE.turret.getError()) < 200
                                ),
                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[3]- TurretNF.INSTANCE.turret.startPosition),

                                shootBalls(
                                        new double[] {0.35, 0.375, 0.4},
                                        new double[] {0, 0},
                                        new double[] {0.4, 0.4},
                                        300,
                                        450
                                )
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),

                followCancelable(paths.intake, 1000),

                new FollowPath(paths.returnn),


                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(
                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[4]),

                                new WaitUntil(() -> (
                                        FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= flywheel_target - 1000)
                                        //&& Math.abs(TurretNF.INSTANCE.turret.getError()) < 200
                                ),

                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[3]),
                                shootBalls(
                                        new double[] {0.35, 0.375, 0.4},
                                        new double[] {0, 0},
                                        new double[] {0.4, 0.4},
                                        300,
                                        450
                                )
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
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


