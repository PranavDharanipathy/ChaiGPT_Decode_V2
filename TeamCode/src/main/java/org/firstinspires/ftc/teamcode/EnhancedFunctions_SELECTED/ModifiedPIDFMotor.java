package org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ModifiedPIDFMotor {

    public DcMotorEx internalMotor;
    private PIDController controller;

    private double kp, ki, kd, kf;

    public ModifiedPIDFMotor(HardwareMap hardwareMap, String deviceName) {

        internalMotor = hardwareMap.get(DcMotorEx.class, deviceName);
        internalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        internalMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        resetEncoder();
    }

    public void resetEncoder() {
        internalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public enum Direction {
        FORWARD,
        REVERSE
    }

    public void setDirection(Direction direction) {
        internalMotor.setDirection(direction == Direction.FORWARD ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
    }

    private double KF_ENABLE_RANGE;

    /// @param KF_ENABLE_RANGE - range that feedforward is used
    public void initialize(double kp, double ki, double kd, double kf, double KF_ENABLE_RANGE) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;

        this.KF_ENABLE_RANGE = KF_ENABLE_RANGE;

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
        double f = Math.abs(pid) > KF_ENABLE_RANGE ? kf : 0;

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