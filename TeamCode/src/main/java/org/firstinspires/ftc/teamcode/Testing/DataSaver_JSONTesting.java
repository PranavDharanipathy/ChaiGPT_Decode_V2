package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.DataSaver_JSON;

@TeleOp(group = "testing")

public class DataSaver_JSONTesting extends OpMode {

    DataSaver_JSON dataSaver;

    int TestData = 1000;

    boolean hasSaved = false;
    @Override
    public void init() {

         dataSaver = new DataSaver_JSON("DataSaverTest");

    }

    @Override
    public void loop() {

        if (!hasSaved) {
            dataSaver.addData("TEST DATA", TestData);
            hasSaved = true;
        }

        else {
            terminateOpModeNow();
        }

    }
}
