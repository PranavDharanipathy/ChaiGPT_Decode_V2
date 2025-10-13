package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.TeleOp.drive.HolonomicDrive;
import org.firstinspires.ftc.teamcode.TeleOp.drive.MaxDriveVoltage;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

public class V2TeleOp extends TeleOpBaseOpMode {

    private HolonomicDrive holonomicDrive;

    @Override
    public void runOpMode() {

        initializeDevices();

        applyComponentTraits();

        //initialize subsystems here
        holonomicDrive = new HolonomicDrive(hardwareMap, Constants.HUB_TYPE.CONTROL_HUB, new MaxDriveVoltage());
        holonomicDrive.setMotors(
                Constants.MapSetterConstants.leftFrontMotorDeviceName,
                Constants.MapSetterConstants.rightFrontMotorDeviceName,
                Constants.MapSetterConstants.leftBackMotorDeviceName,
                Constants.MapSetterConstants.rightBackMotorDeviceName
        );
        holonomicDrive.reverseMotors(
                Constants.DriveConstants.HolonomicDriveSidesReversed.FRONT_MOTOR.getStringValue(),
                Constants.DriveConstants.HolonomicDriveSidesReversed.BACK_MOTOR.getStringValue()
        );
        holonomicDrive.setTurnPolarities(
                Constants.DriveConstants.leftFrontTurnPolarity,
                Constants.DriveConstants.rightFrontTurnPolarity,
                Constants.DriveConstants.leftBackTurnPolarity,
                Constants.DriveConstants.rightBackTurnPolarity
        );

        //setup lynx module
        setUpLynxModule();

        if (isStopRequested()) return;
        waitForStart();

        //run robot reset
        RobotResetter robotReset = new PostAutonomousRobotReset();

        while (opModeIsActive()) {

            TickrateChecker.startOfLoop();

            // clear data at start of loop
            clearCacheOfLynxModule();

            holonomicDrive.drive(controller1.left_stick_y(), controller1.left_stick_x(), controller1.right_stick_x());

            TickrateChecker.endOfLoop();

            telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());
            telemetry.addData("(Predicted) Current CPU Usage", TickrateChecker.getTimeBasedCpuUsagePrediction());
            telemetry.update();

            if(isStopRequested()) {
                //end
                closeLynxModule();
            }
        }
    }
}
