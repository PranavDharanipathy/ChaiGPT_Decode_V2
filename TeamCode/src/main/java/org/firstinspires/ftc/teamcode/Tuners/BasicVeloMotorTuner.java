package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;

@Config
@TeleOp (group = "tuning")
public class BasicVeloMotorTuner extends OpMode {

    public static String motorName = "";

    private BasicVeloMotor motor;

    public static double VELOCITY = 500;

    public static double KP = 0;
    public static double KI = 0;
    public static double KD = 0;
    public static double KF = 0;

    @Override
    public void init() {

        motor = new BasicVeloMotor(hardwareMap, motorName);
    }

    @Override
    public void loop() {

        motor.setVelocityPIDFCoefficients(KP, KI, KD, KF); //coeffs are updated

        motor.setVelocity(VELOCITY);

        telemetry.addData("velocity", motor.getVelocity());
        telemetry.addData("power", motor.getPower());
        telemetry.update();
    }
}
