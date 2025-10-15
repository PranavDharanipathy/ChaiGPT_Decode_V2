package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Command;

public abstract class WaitCommand implements Command {


    // Wait function
    public static void Wait(long ms) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < ms) {
            // do nothing/wait
        }
    }
}
