package org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PIDFMotor {

    public DcMotorEx internalMotor;
    private PIDController controller;

    private double kp, ki, kd, kf, TICKS_PER_DEGREE;

    public PIDFMotor(HardwareMap hardwareMap, String deviceName) {

        internalMotor = hardwareMap.get(DcMotorEx.class, deviceName);

        internalMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public enum Direction {
        FORWARD,
        REVERSE
    }

    public void setDirection(Direction direction) {
        internalMotor.setDirection(direction == Direction.FORWARD ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
    }

    /// @param GEAR_RATIO - output divided by input
    public void initialize(double kp, double ki, double kd, double kf, double TICKS_PER_REV, double GEAR_RATIO) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;

        TICKS_PER_DEGREE = TICKS_PER_REV / (360 * GEAR_RATIO);

        controller = new PIDController(kp, ki, kd);
    }

    /// simpler to call
    public void setPosition(int targetPosition) {
        internalMotor.setTargetPosition(targetPosition);
    }

    /// runs a single instance of PIDF loop
    public void update() {

        controller.setPID(kp, ki, kd);

        int targetPosition = internalMotor.getTargetPosition();

        double pid = controller.calculate(internalMotor.getCurrentPosition(), targetPosition);
        double f = kf * Math.cos(Math.toRadians(targetPosition / TICKS_PER_DEGREE));

        internalMotor.setPower(pid + f);

    }

    /// used for tuning
    public void updateCoefficients(double kp, double ki, double kd, double kf) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }

}
