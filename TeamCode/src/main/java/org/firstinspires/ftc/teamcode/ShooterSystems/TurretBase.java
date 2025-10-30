package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class TurretBase {

    private CRServoImplEx leftTurretBase, rightTurretBase;
    private Encoder encoder;

    private PIDController controller;

    private double multiplier = 1;

    private double kp, ki, kd, kf;

    public TurretBase(HardwareMap hardwareMap, String leftCRServoDeviceName, String rightCRServoDeviceName) {

        leftTurretBase = hardwareMap.get(CRServoImplEx.class, Constants.MapSetterConstants.turretBaseLeftServoDeviceName);
        rightTurretBase = hardwareMap.get(CRServoImplEx.class, Constants.MapSetterConstants.turretBaseRightServoDeviceName);

        leftTurretBase.setDirection(Constants.TURRET_BASE_DIRECTIONS[0]);
        rightTurretBase.setDirection(Constants.TURRET_BASE_DIRECTIONS[1]);

        leftTurretBase.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightTurretBase.setPwmRange(new PwmControl.PwmRange(500, 2500));

        encoder = new Encoder(hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.turretExternalEncoderMotorPairName));
        encoder.setDirection(Encoder.Direction.FORWARD);

    }

    public void setPIDFCoefficients(double kp, double ki, double kd, double kf) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;

        controller = new PIDController(kp, ki, kd);
    }

    private double targetPosition = 0;

    private double MAX_I = Double.MAX_VALUE;
    private double MIN_I = -Double.MAX_VALUE;

    public void setIConstraints(double min_i, double max_i) {

        MIN_I = min_i;
        MAX_I = max_i;
    }

    public void setPosition(double position) {
        targetPosition = position;
    }

    public void setMultiplier(double multiplier){
        this.multiplier = multiplier;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getCurrentPosition() {
        return encoder.getCurrentPosition();
    }

    public void update() {

        controller.setPID(kp, ki, kd);
        controller.setIntegrationBounds(MIN_I, MAX_I);

        double pid = controller.calculate(getCurrentPosition(), targetPosition);
        double f = usingFeedforward ? (targetPosition - getCurrentPosition() >= 0 ? kf : -kf) : 0;

        double power = pid + f;

        leftTurretBase.setPower(power * multiplier);
        rightTurretBase.setPower(power * multiplier);
    }

    private boolean usingFeedforward = true;

    public void setUsingFeedforwardState(boolean usingFeedforward) {
        this.usingFeedforward = usingFeedforward;
    }

    public double $getPositionError() {
        return controller.getPositionError();
    }

    /// used for tuning
    public void updateCoefficients(double kp, double ki, double kd, double kf) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }

}