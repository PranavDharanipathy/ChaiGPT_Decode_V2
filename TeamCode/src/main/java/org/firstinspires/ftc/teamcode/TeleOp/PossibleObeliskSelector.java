package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.SimpleDataFile_XML;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class PossibleObeliskSelector extends Subsystem {

    private SimpleDataFile_XML obeliskDataFile;

    private Obelisk currentObelisk;

    private BetterGamepad controller2;

    private int aprilTagNumberFromData;

    private ElapsedTime keybindTimer = new ElapsedTime();

    public void provideComponents(HardwareMap hardwareMap, BetterGamepad controller2) {

        obeliskDataFile = new SimpleDataFile_XML(Constants.IOConstants.OBELISK_XML_FILE_NAME, hardwareMap.appContext);
        int aprilTagNumber = obeliskDataFile.loadData(Constants.IOConstants.OBELISK_XML_DATA_KEY, Constants.IOConstants.OBELISK_XML_DEFAULT_KEY);

        aprilTagNumberFromData = aprilTagNumber;
        currentObelisk = new Obelisk(Obelisk.getFromAprilTagNumber(aprilTagNumber));

        this.controller2 = controller2;
    }

    /// Gets the Obelisk Mode set either by the driver or the robot
    /// <p>
    /// - For telemetry.
    public Obelisk.OBELISK getCurrentObelisk() {
        return currentObelisk.getObelisk();
    }

    @Override
    public void update() {

        aprilTagNumberFromData = obeliskDataFile.loadData(Constants.IOConstants.OBELISK_XML_DATA_KEY, Constants.IOConstants.OBELISK_XML_DEFAULT_KEY);

        // save data if changes are made
        if (currentObelisk.getObelisk().getAprilTagNumber() != aprilTagNumberFromData) {
            obeliskDataFile.saveData(Constants.IOConstants.OBELISK_XML_DATA_KEY, currentObelisk.getObelisk().getAprilTagNumber());
        }

        // if current obelisk code invalid
        if (currentObelisk.getObelisk().getAprilTagNumber() == -1) {
            controller2.rumble(Constants.OBELISK_SELECTION_KEYBIND_TIME);
        }
        else controller2.stopRumble();

        keybindProcessing();
    }

    //used to represent a unit of the obelisk code
    private String Pkey = "p";
    private String Gkey = "g";

    private String obeliskModeKeybind = null;

    private void keybindProcessing() {

        // button a -> green
        // button b -> purple

        // if driver takes too long to enter the keybind, then code cancels the entry
        if (keybindTimer.milliseconds() > Constants.OBELISK_SELECTION_KEYBIND_TIME && obeliskModeKeybind != null) {
            obeliskModeKeybind = null;
        }

        if (controller2.aHasJustBeenPressed || controller2.bHasJustBeenPressed) {

            if (obeliskModeKeybind == null) keybindTimer.reset();

            if (controller2.aHasJustBeenPressed) {
                obeliskModeKeybind += Pkey;
            }
            else { // 'controller2.bHasJustBeenPressed' must be true
                obeliskModeKeybind += Gkey;
            }

            if (obeliskModeKeybind.length() == 3) { //each obelisk code is 3

                currentObelisk.setObelisk(convertToOBELISK(obeliskModeKeybind));

                obeliskModeKeybind = null;
            }
        }
    }

    /// will only convert if valid keybind was provided, otherwise it will keep whatever code was already there
    private Obelisk.OBELISK convertToOBELISK(String stringKeybind) {

        Obelisk.OBELISK codeInOBELISKForm;

        switch (stringKeybind) {

            case "gpp": //tag: 21
                codeInOBELISKForm = Obelisk.OBELISK.GPP;
                break;

            case "pgp": //tag: 22
                codeInOBELISKForm = Obelisk.OBELISK.PGP;
                break;

            case "ppg": //tag: 23
                codeInOBELISKForm = Obelisk.OBELISK.PPG;
                break;

            default: //if an invalid obelisk code is entered, it just keeps the current code
                codeInOBELISKForm = currentObelisk.getObelisk();
        }

        return codeInOBELISKForm;
    }
}
