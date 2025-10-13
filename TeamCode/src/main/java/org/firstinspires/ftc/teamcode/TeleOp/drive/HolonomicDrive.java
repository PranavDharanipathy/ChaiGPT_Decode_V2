package org.firstinspires.ftc.teamcode.TeleOp.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.VMotor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Holonomic Drive designed for TeleOp for mecanum drivetrains.
 * <p>
 * All motors are instances of {@link VMotor}
 **/
@SuppressWarnings("unused")
public class HolonomicDrive {

    private HardwareMap hardwareMap;

    private VMotor left_front, right_front, left_back, right_back;
    private final ArrayList<String> motorNames = new ArrayList<>();

    private final Constants.HUB_TYPE driveTrainMotorHub;

    private double maxVoltage;

    private final boolean automaticMaxVoltageCalculationMode;

    public HolonomicDrive(HardwareMap hardwareMap, Constants.HUB_TYPE driveTrainMotorHub, MaxDriveVoltage maxVoltage) {

        this.hardwareMap = hardwareMap;

        this.driveTrainMotorHub = driveTrainMotorHub;

        automaticMaxVoltageCalculationMode = maxVoltage.getVoltageAllowed();

        if (automaticMaxVoltageCalculationMode) this.maxVoltage = maxVoltage.getVoltageValue();
    }

    /// ***Automatically determines max drive voltage.***
    public HolonomicDrive(HardwareMap hardwareMap, Constants.HUB_TYPE driveTrainMotorHub) {

        this (hardwareMap, driveTrainMotorHub, new MaxDriveVoltage()); //set to automatically determine max drive voltage
    }

    public void setMotors(String left_front, String right_front, String left_back, String right_back) {

        this.left_front = new VMotor(hardwareMap, left_front, driveTrainMotorHub);
        this.right_front = new VMotor(hardwareMap, right_front, driveTrainMotorHub);
        this.left_back = new VMotor(hardwareMap, left_back, driveTrainMotorHub);
        this.right_back = new VMotor(hardwareMap, right_back, driveTrainMotorHub);

        this.left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //DETERMINE MAX VOLTAGE USING A DRIVE MOTOR IF WANTED
        if (automaticMaxVoltageCalculationMode) {
            maxVoltage = this.left_front.internalBatteryVoltageSensor.getVoltage();
        }

        motorNames.add(left_front); // left_front: index 0
        motorNames.add(right_front); // right_front: index 1
        motorNames.add(left_back); // left_back: index 2
        motorNames.add(right_back); // right_back: index 3
    }

    /// @throws IllegalArgumentException Given motor name has not been applied to a motor
    private void reverseDcMotor(String motorName) {

        if (!motorNames.contains(motorName)) throw new IllegalArgumentException("Given motor name has not been applied to a motor.");

        int motorLocation = motorNames.indexOf(motorName);

        /*left_front*/ if (motorLocation == 0) left_front.setDirection(DcMotor.Direction.REVERSE);
        /*right_front*/ else if (motorLocation == 1) right_front.setDirection(DcMotor.Direction.REVERSE);
        /*left_back*/ else if (motorLocation == 2) left_back.setDirection(DcMotor.Direction.REVERSE);
        /*right_back*/ else if (motorLocation == 3) right_back.setDirection(DcMotor.Direction.REVERSE);
    }

    /// @throws IllegalArgumentException Cannot reverse more than 4 motors
    /// @throws IllegalArgumentException Cannot reverse 0 motors
    public void reverseMotors(String... motors) {

        if (motors.length > 4) throw new IllegalArgumentException("Reversing more than 4 motors on your drivetrain is not aura.");
        else if (motors.length == 0) throw new IllegalArgumentException("Cannot reverse 0 motors");

        for (String motor : motors) reverseDcMotor(motor);
    }

    private double lfPower;
    private double rfPower;
    private double lbPower;
    private double rbPower;

    /**
     * HOLONOMIC DRIVE
     * @param forward - joystick input
     * @param strafe - joystick input
     * @param turn - joystick input
     *
     * <p>|<p>
     * If any of the parameters are below a certain threshold, they will be ignored completely.
     * This  leads to forward, strafe, and turn, sometimes being completely 'PURE' due to the other not being above the threshold.
     *
     * <p>|<p>
     * PURE = only a certain feature is used in 2D, non-rotational movements, so strafe may be ignored or forward may be ignored with the other being used.
     * <p> With turn, it's applied after the non-rotational movements but only if it itself is above the set threshold.
     */
    public void drive(float forward, float strafe, float turn) {

        float f = Math.abs(forward);
        float s = Math.abs(strafe);
        float t = Math.abs(turn);

        //default is no power - but the power is not yet given to the motors
        lfPower = 0;
        rfPower = 0;
        lbPower = 0;
        rbPower = 0;

        /*
         *     s
         *  .-----*
         *   \    |
         *    \   |
         * hyp \  | f
         *      \ |
         *       \|
         *
         */

        //finds what quadrant the bot is moving into as if it's at the origin (0,0)
        QUADRANT quadrant = pickQuadrant(strafe, forward);

        //doesn't use StrictMath because it doesn't need that level of accuracy with regards for the cost of processing speed
        double hyp = Math.sqrt(Math.pow(s, 2) + Math.pow(f, 2)); //hypotenuse

        //law of cosines with SSS
        // c^2 = a^2 + b^2 - 2ab * cos(C degrees)
        // cos(C degrees) = (a^2 + b^2 - c^2) / 2ab

        //solving for A in degrees
        // cos^-1((a^2 + b^2 - c^2) / 2ab))
        double rawDriveAngle = Math.toDegrees(Math.acos(Math.toRadians((Math.pow(f, 2) + Math.pow(hyp, 2) - Math.pow(s, 2)) / (2 * f * hyp))));
        /* A' */ double aidWheelUsableAngle = rawDriveAngle - 45;

        // whichever direction (forward, strafe) has more power
        boolean powerComparison = f > s;

        // records if f, s, and t are below a certain threshold or not
        List<Boolean> fstAboveMinimums = Arrays.asList(f > 0.02, s > 0.02, t > 0.02);

        if (fstAboveMinimums.contains(true)) { //if the drive joysticks have actually been moved above the threshold

            // if strafe is below margin, then it's ignored
            if (!fstAboveMinimums.get(1) && fstAboveMinimums.get(0)) {
                lfPower = forward;
                rfPower = forward;
                lbPower = forward;
                rbPower = forward;
            }

            // if forward is below margin, then it's ignored
            else if (!fstAboveMinimums.get(0) && fstAboveMinimums.get(1)) {
                //change polarity based on your drivetrain
                lfPower = -strafe;
                rfPower = strafe;
                lbPower = strafe;
                rbPower = -strafe;
            }

            // if both forward and strafe are above margin
            else if (fstAboveMinimums.get(0) && fstAboveMinimums.get(1)) {

                if (powerComparison) { //mainly forward

                    double strafeAmount = hyp * Math.sin(Math.toRadians(aidWheelUsableAngle));

                    setBasePowers(quadrant, forward, "forward");

                    //aid powers
                    setAidPowers(quadrant, strafeAmount, rawDriveAngle, "forward");
                }
                else /*powerComparison is false*/ { //mainly strafe

                    double forwardAmount = hyp * Math.cos(Math.toRadians(aidWheelUsableAngle));

                    setBasePowers(quadrant, strafe, "strafe");

                    //aid powers
                    setAidPowers(quadrant, forwardAmount, rawDriveAngle, "strafe");
                }
            }

            // if turn is above threshold, then it's APPLIED
            if (fstAboveMinimums.get(2)) {
                //change polarity based on your drivetrain
                lfPower += turn * lfTurnPolarityValue;
                rfPower += turn * rfTurnPolarityValue;
                lbPower += turn * lbTurnPolarityValue;
                rbPower += turn * rbTurnPolarityValue;
            }
        }

        left_front.setVolts(lfPower * maxVoltage);
        right_front.setVolts(rfPower * maxVoltage);
        left_back.setVolts(lbPower * maxVoltage);
        right_back.setVolts(rbPower * maxVoltage);
    }

    public enum TurnPolarity {

        POSITIVE(1), NEGATIVE(-1);

        private int polarity;

        TurnPolarity(int polarity) {
            this.polarity = polarity;
        }

        public int getValue() {
            return polarity;
        }
    }

    // Default polarities during turns
    private int lfTurnPolarityValue = -1;
    private int rfTurnPolarityValue = 1;
    private int lbTurnPolarityValue = -1;
    private int rbTurnPolarityValue = 1;

    /// Setting polarities of motors during turns
    public void setTurnPolarities(TurnPolarity leftFrontPolarity, TurnPolarity rightFrontPolarity, TurnPolarity leftBackPolarity, TurnPolarity rightBackPolarity) {

        lfTurnPolarityValue = leftFrontPolarity.getValue();
        rfTurnPolarityValue = rightFrontPolarity.getValue();
        lbTurnPolarityValue = leftBackPolarity.getValue();
        rbTurnPolarityValue = rightBackPolarity.getValue();
    }

    /// @param amt Isn't produced by joystick but instead calculated.
    /// @throws IllegalArgumentException Can only provide a 'type' of 'forward' or 'strafe'
    private void setAidPowers(QUADRANT quadrant, double amt, double rawDriveAngle, String type) {

        /* Consider bot "forward" and "strafe"
         *
         * QUADRANT I     (+,+)
         * QUADRANT II    (-,+)
         * QUADRANT III   (-,-)
         * QUADRANT IV    (+,-)
         */

        if (type.equals("forward")) {

            if (quadrant == QUADRANT.I || quadrant == QUADRANT.III) {
                rfPower = lbPower = quadrant == QUADRANT.III ? -amt : amt;
                reversePowersIfNeeded(POWER_TYPE.RF, POWER_TYPE.LB, rawDriveAngle);
            }
            else if (quadrant == QUADRANT.II || quadrant == QUADRANT.IV) {
                lfPower = rbPower = quadrant == QUADRANT.II ? -amt : amt;
                reversePowersIfNeeded(POWER_TYPE.LF, POWER_TYPE.RB, rawDriveAngle);
            }
        }
        else if (type.equals("strafe")) {

            if (quadrant == QUADRANT.I || quadrant == QUADRANT.III) {
                rfPower = lbPower = quadrant == QUADRANT.III ? -amt : amt;
                reversePowersIfNeeded(POWER_TYPE.RF, POWER_TYPE.LB, rawDriveAngle);
            }
            else if (quadrant == QUADRANT.II || quadrant == QUADRANT.IV) {
                lfPower = rbPower = quadrant == QUADRANT.IV ? -amt : amt;
                reversePowersIfNeeded(POWER_TYPE.LF, POWER_TYPE.RB, rawDriveAngle);
            }
        }
        else throw new IllegalArgumentException("Can only provide a 'type' of 'forward' or 'strafe'!");
    }

    /// For reversing.
    public enum POWER_TYPE {
        LF, RF, LB, RB
    }

    private void reversePowersIfNeeded(POWER_TYPE power1, POWER_TYPE power2, double rawDriveAngle) {

        if (rawDriveAngle < 45) {

            POWER_TYPE[] powers = {power1, power2};

            //'ey I'm leavin' this, I know I can make it into a for-each.
            for (int powerIndex = 0; powerIndex < 2; powerIndex++) {

                switch (powers[powerIndex]) {

                    case LF:
                        lfPower = -lfPower;

                    case RF:
                        rfPower = -rfPower;

                    case LB:
                        lbPower = -lbPower;

                    case RB:
                        rbPower = -rbPower;
                }
            }
        }
    }

    /// @throws IllegalArgumentException Can only provide a 'type' of 'forward' or 'strafe'
    private void setBasePowers(QUADRANT quadrant, float amt, String type) {

        if (type.equals("forward")) {

            if (quadrant == QUADRANT.I || quadrant == QUADRANT.III) {
                lfPower = rbPower = amt;
            } else if (quadrant == QUADRANT.II || quadrant == QUADRANT.IV) {
                rfPower = lbPower = amt;
            }
        }
        else if (type.equals("strafe")) {

            if (quadrant == QUADRANT.I || quadrant == QUADRANT.IV) {
                lfPower = rbPower = amt;
            } else if (quadrant == QUADRANT.II || quadrant == QUADRANT.III) {
                rfPower = lbPower = amt;
            }
        }
        else throw new IllegalArgumentException("Can only provide a 'type' of 'forward' or 'strafe'!");
    }

    public enum QUADRANT {
        I, II, III, IV,
        UNMOVED
    }

    /** MAINLY USED INTERNALLY
     * <p>
     * Picks a quadrant that the bot will move into assuming that the bot's at (0,0).
     * @return quadrant (I, II, III, IV, UNMOVED)
     **/
    public QUADRANT pickQuadrant(float x, float y) {

        boolean quadI = (x>1 && y>1);
        boolean quadII = (x<1 && y>1);
        boolean quadIII = (x<1 && y<1);
        boolean quadIV = (x>1 && y<1);

        return quadI ? QUADRANT.I : (quadII ? QUADRANT.II : (quadIII ? QUADRANT.III : (quadIV ? QUADRANT.IV : QUADRANT.UNMOVED)));
    }
}