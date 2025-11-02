package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.chaigptrobotics.shenanigans.Peak;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

/**
  * Using odometry and the goBILDA Pinpoint Odometry Computer, we calculate the rate at which
  * the robot's yaw is changing as well as the direction and speed that the robot is moving
  * at in relation to the goal.
  * **/
 @Peak
public class DrivetrainVectorUtilityObject {

     private GoBildaPinpointDriver pinpoint;

     private Rev9AxisImu rev9AxisIMU;

     public DrivetrainVectorUtilityObject(HardwareMap hardwareMap) {

         pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.MapSetterConstants.pinpointOdometryComputerDeviceName);
         pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

         double mmPerTick = MecanumDrive.PARAMS.inPerTick * 25.4;
         pinpoint.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);

         pinpoint.setOffsets(
                 mmPerTick * PinpointLocalizer.PARAMS.parYTicks,
                 mmPerTick * PinpointLocalizer.PARAMS.perpXTicks, DistanceUnit.MM);

         pinpoint.setEncoderDirections(
                 PinpointLocalizer.INITIAL_PAR_DIRECTION,
                 PinpointLocalizer.INITIAL_PERP_DIRECTION
                 );

         pinpoint.resetPosAndIMU();

         rev9AxisIMU = hardwareMap.get(Rev9AxisImu.class, Constants.MapSetterConstants.rev9AxisIMUDeviceName);
         rev9AxisIMU.initialize(Constants.IMUConstants.getRev9AxisIMUParams());
     }

    private Orientation orientation;

    private double roll, pitch, yaw;

     public void update() {

         orientation = Constants.IMUConstants.getRev9AxisIMUOrientationStats(rev9AxisIMU);

         YawPitchRollAngles angles = rev9AxisIMU.getRobotYawPitchRollAngles();

         yaw = angles.getYaw();
         pitch = angles.getPitch();
         roll = angles.getRoll();

         
     }
}
