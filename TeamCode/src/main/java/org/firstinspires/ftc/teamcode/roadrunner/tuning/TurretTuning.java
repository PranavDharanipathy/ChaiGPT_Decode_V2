package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;
import org.firstinspires.ftc.teamcode.ShooterSystems.ExtremePrecisionFlywheel;
import org.firstinspires.ftc.teamcode.ShooterSystems.HoodAngler;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;


@Config
@TeleOp(name = "Turret Tuning", group="tuning")
public class TurretTuning extends OpMode {
    public static double TURRET_POSITION = 0;

    //0.11 = hood fully forward/facing down

    //0.9 = hood fully back(facing up)

    public static double[] HOOD_ANGLES= {0.11, 0.9};

    public class RobotElements {
        public class AllUpdate implements Action {
            private ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (timer.milliseconds() > Constants.FLYWHEEL_PIDFVAS_LOOP_TIME) {
                    flywheel.update();
                    timer.reset();
                }

                turret.update();

                telemetry.addData("turret target position", turret.getTargetPosition());
                telemetry.addData("turret current position", turret.getCurrentPosition());
                telemetry.update();

                return true;
            }
        }

        public class FlywheelWaitVel implements Action {
            private ElapsedTime timer = new ElapsedTime();

            private double minimumTime;

            public FlywheelWaitVel(double minimumTime) {
                this.minimumTime = minimumTime;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
                telemetry.update();
                return !(timer.seconds() >= minimumTime && flywheel.getFrontendCalculatedVelocity() > 25500 && flywheel.getLastFrontendCalculatedVelocity() > 25500);
            }


        }


        public Action updates() {
            return new RobotElements.AllUpdate();
        }

        public InstantAction setFlywheelToCloseSideVelocity() {

            return new InstantAction(() -> flywheel.setVelocity(28600, true));
        }

        public InstantAction stopFlywheel() {
            return new InstantAction(() -> flywheel.setVelocity(0, true));
        }


        //transfer
        public InstantAction antiTransfer() {
            return new InstantAction(() -> transfer.setVelocity(Constants.ANTI_TRANSFER_VELOCITY));
        }

        public InstantAction transferArtifact() {
            return new InstantAction(() -> transfer.setVelocity(Constants.TRANSFER_VELOCITY));
        }


        public Action waitFlywheelVel(double minimumTime) {
            return new FlywheelWaitVel(minimumTime);
        }

        //public double turretStartPosition;
        public Action Shoot() {

            return new SequentialAction(
                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2])),
                    new InstantAction(() -> hoodAngler.setPosition(HOOD_ANGLES[0])),
                    waitFlywheelVel(3),
                    transferArtifact()
            );
        }

    }
        public double turretStartPosition;

    TurretBase turret;
    ExtremePrecisionFlywheel flywheel;
    BasicVeloMotor transfer;
    HoodAngler hoodAngler;




    @Override
    public void init() {
        turret = new TurretBase(hardwareMap);

        flywheel = new ExtremePrecisionFlywheel(
                hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.leftFlywheelMotorDeviceName),
                hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.rightFlywheelMotorDeviceName)
        );

        transfer = new BasicVeloMotor(hardwareMap, Constants.MapSetterConstants.transferMotorDeviceName);
        hoodAngler = new HoodAngler(hardwareMap, Constants.MapSetterConstants.hoodAnglerLeftServoDeviceName, Constants.MapSetterConstants.hoodAnglerRightServoDeviceName);





    }
        @Override
        public void loop() {

            final RobotElements robot = new RobotElements();
            Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(-225));
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

            new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITION));

                    robot.setFlywheelToCloseSideVelocity();

                    //IF NEEDED, DRIVE TO LOCATION and then shoot




            turretStartPosition = turret.getCurrentPosition();

            //SHOOT!
            telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
            telemetry.update();


        }
    }


