package org.firstinspires.ftc.teamcode.TeleOp.drive;

public class MaxDriveVoltage {

    private Double voltage;

    /// Max voltage set
    public MaxDriveVoltage(double voltage) {
        this.voltage = voltage;
    }

    /// Max voltage set to current hub voltage
    public MaxDriveVoltage() {
        voltage = null;
    }

    public double getVoltageValue() {
        return voltage;
    }

    /// @return 'true' if voltage is allowed and 'false' if not allowed and automatically set
    public boolean getVoltageAllowed() {
        return voltage != null;
    }
}