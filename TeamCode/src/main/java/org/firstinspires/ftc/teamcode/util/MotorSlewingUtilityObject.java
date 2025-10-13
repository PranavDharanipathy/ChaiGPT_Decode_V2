package org.firstinspires.ftc.teamcode.util;

public class MotorSlewingUtilityObject {

    private double slewSlope = 0; //rate of slewing motor
    private double slew = 0;

    private double totalTicksSlewed = 0;

    private double pastDeltaVelocity = 0;

    /// @param deltaVelocity The change in velocity.
    /// @param periodOfTicks The total number of ticks that the change in velocity should happen over.
    /// @param allowSlewReset Resets totalTicksSlewed.
    public void updateSlew(double deltaVelocity, int periodOfTicks, boolean allowSlewReset) {

        if (allowSlewReset && deltaVelocity != pastDeltaVelocity) {

            slew = 0;
            totalTicksSlewed = 0;
        }

        pastDeltaVelocity = deltaVelocity;

        slewSlope = deltaVelocity / periodOfTicks;

        slew += slewSlope;
    }

    public double getSlewRate() {
        return slewSlope;
    }

    public double getSlew(boolean appliedToMotor) {

        if (appliedToMotor) totalTicksSlewed++;

        return slew;
    }

    public double get$TicksSlewed() {
        return totalTicksSlewed;
    }
}
