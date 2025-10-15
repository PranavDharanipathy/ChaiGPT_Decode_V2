package org.firstinspires.ftc.teamcode.TeleOp.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SingleEncoderDriveSide {

    private final DcMotor base; //has encoder
    private final DcMotor aid; //no encoder

    /// @param deviceNames <p>- index 1: front <p>- index 2: back
    public SingleEncoderDriveSide(HardwareMap hardwareMap, String[] deviceNames) {

        base = hardwareMap.get(DcMotor.class, deviceNames[0]);
        aid = hardwareMap.get(DcMotor.class, deviceNames[1]);

        base.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aid.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        base.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aid.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /// @param directions <p>- index 1: front <p>- index 2: back
    public void setDirection(DcMotorSimple.Direction directions[]) {
        base.setDirection(directions[0]);
        aid.setDirection(directions[1]);
    }

    public void setVelocityPIDFCoefficients(double kp, double ki, double kd, double kf) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }

    public double getPower() {
        return base.getPower();
    }

    private ElapsedTime timer;

    private double prevTime = 0, currTime = 0;

    private double prevError = 0, error = 0;

    private double prevPosition = 0, position = 0;

    private double kp, ki, kd, kf;

    private double p, i, d, f;

    public void update(double targetVelocity) {


        currTime = timer.milliseconds();
        double dt = currTime - prevTime;

        position = base.getCurrentPosition();

        double deltaTicks = position - prevPosition;

        double currentVelocity = deltaTicks / dt;

        error = targetVelocity - currentVelocity;

        p = kp * error;

        i += ki * error / dt;

        d = kd * (error - prevError) / dt;

        f = kf;

        double power = p + i + d + f;

        base.setPower(power);
        aid.setPower(power);

        prevPosition = position;
        prevError = error;
        prevTime = currTime;
    }
}
