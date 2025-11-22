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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.AutonomousBaseOpMode;
import org.firstinspires.ftc.teamcode.Auton.TwelveBallAuto_BLUE_CLOSE;
import org.firstinspires.ftc.teamcode.Auton.TwelveBallAuto_BLUE_FAR;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.ShooterSystems.ExtremePrecisionFlywheel;
import org.firstinspires.ftc.teamcode.ShooterSystems.HoodAngler;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;

@Config
@Autonomous(name = "TurretPosition")
public class TurretPosition extends AutonomousBaseOpMode {

    public volatile Telemetry telemetry;

    public volatile BetterGamepad controller1;
    public volatile BetterGamepad controller2;

    public volatile BasicVeloMotor intake;
    public volatile BasicVeloMotor transfer;
    public volatile AdafruitBeambreakSensor intakeBeambreak, transferBeambreak;

    public volatile TurretBase turret;

    public volatile HoodAngler hoodAngler;

    public volatile ExtremePrecisionFlywheel flywheel;

    public double[] TURRET_POSITIONS = {5800, 5000, 5800};

    public double[] HOOD_ANGLING = {0.49, 0.04, 0.06};

    public class RobotElements {

        public InstantAction setFlywheelToCloseSideVelocity() {

            return new InstantAction(() -> flywheel.setVelocity(25600, true));
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

        //intake
        public InstantAction reverseIntake() {
            return new InstantAction(() -> intake.setVelocity(Constants.REVERSE_INTAKE_VELOCITY));
        }


        public InstantAction intake() {
            return new InstantAction(() -> intake.setVelocity(Constants.BASE_INTAKE_VELOCITY));
        }

        public InstantAction fact1() {
            return new InstantAction(() -> telemetry.speak("Did you know that A crocodile cannot stick its tongue out?"));
        }

        public double turretStartPosition;


    }

    public double turretStartPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        init();

        final RobotElements robot = new RobotElements();

        turret.setPosition(turretStartPosition + TURRET_POSITIONS[0]);

        waitForStart();


    }




}
