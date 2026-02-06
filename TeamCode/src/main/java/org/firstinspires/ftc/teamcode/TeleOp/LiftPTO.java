package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

/// Uses an Axon Max Servo
public class LiftPTO {

    private Servo ptoServo;

    public LiftPTO(HardwareMap hardwareMap) {

        ptoServo = hardwareMap.get(Servo.class, Constants.MapSetterConstants.liftPTOServoDeviceName);
        ptoServo.setDirection(Constants.LIFT_PTO_SERVO_DIRECTION);
    }

    /// The PTOState of the LiftPTO will be set to this if the position is not that of ENGAGE or DISENGAGE.
    public static double INVALID_STATE_POSITION = -99999;

    public enum PTOState {

        ENGAGE(Constants.LIFT_PTO_ENGAGE_POSITION), DISENGAGE(Constants.LIFT_PTO_DISENGAGE_POSITION), INVALID(INVALID_STATE_POSITION);

        private double position;

        PTOState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }

        public static PTOState getState(double position) {
            return position == ENGAGE.position
                    ? ENGAGE
                    : (
                        position == DISENGAGE.position
                                ? DISENGAGE
                                : INVALID
                    );
        }
    }

    public void setState(PTOState state) {

        if (state == PTOState.INVALID) throw new IllegalArgumentException("Not allowed to set servo to state INVALID!");

        ptoServo.setPosition(state.getPosition());
    }

    public PTOState getState() {
        double position = ptoServo.getPosition();
        return PTOState.getState(position);
    }

    public double getPosition() {
        return ptoServo.getPosition();
    }
}