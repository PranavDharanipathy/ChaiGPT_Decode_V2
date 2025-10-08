package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.PIVoltageMotor;
import org.firstinspires.ftc.teamcode.util.VMotor;

@Config
@TeleOp (group = "tuning")
public class PIVoltageMotorTuner extends OpMode {

    private PIVoltageMotor motor;

    public static String motorName = "";
    public static Constants.HUB_TYPE hubType = Constants.HUB_TYPE.CONTROL_HUB;

    public static double VOLTS = 12;
    public static double VELOCITY = 0;

    public static double KP = 0;
    public static double KI = 0;

    @Override
    public void init() {

        motor = new PIVoltageMotor(hardwareMap, motorName, hubType);
    }

    @Override
    public void loop() {

        motor.setVelocityPICoefficients(KP, KI);

        if (VELOCITY == 0) motor.setVolts(VOLTS);
        else motor.setVelocity(VELOCITY);

        telemetry.addData("velocity", motor.getVelocity());
        telemetry.addData("power", motor.getPower());
        telemetry.update();
    }
}
