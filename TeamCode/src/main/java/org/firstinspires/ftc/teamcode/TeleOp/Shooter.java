package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.chaigptrobotics.shenanigans.Peak;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.ShooterSystems.ExtremePrecisionFlywheel;
import org.firstinspires.ftc.teamcode.ShooterSystems.Goal;
import org.firstinspires.ftc.teamcode.ShooterSystems.HoodAngler;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.roadrunner.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Rev9AxisImuWrapped;
import org.firstinspires.ftc.teamcode.util.SubsystemInternal;

@Peak
public class Shooter implements SubsystemInternal {

    private BetterGamepad controller1, controller2;

    private CustomMecanumDrive customDrive;

    public ExtremePrecisionFlywheel flywheel;

    public TurretBase turret;

    public HoodAngler hoodAngler;

    private Rev9AxisImuWrapped rev9AxisImuWrapped;

    public void provideComponents(ExtremePrecisionFlywheel flywheel, TurretBase turret, HoodAngler hoodAngler, CustomMecanumDrive customDrive, Rev9AxisImu rev9AxisImu, BetterGamepad controller1, BetterGamepad controller2) {

        rev9AxisImuWrapped = new Rev9AxisImuWrapped(rev9AxisImu);

        this.customDrive = customDrive;

        this.flywheel = flywheel;

        this.turret = turret;

        this.hoodAngler = hoodAngler;

        this.controller1 = controller1;
        this.controller2 = controller2;

    }

    enum ZONE {
        CLOSE, FAR
    }

    private ZONE flywheelTargetVelocityZone = ZONE.FAR;

    private double turretStartPosition;

    private Goal.GoalCoordinates goalCoordinates;

    private double turretAngularOffset = ShooterInformation.ShooterConstants.TURRET_ANGULAR_OFFSET;

    public void start(Goal.GoalCoordinates goalCoordinates) {

        this.goalCoordinates = goalCoordinates;

        if (goalCoordinates == Goal.GoalCoordinates.BLUE) turretAngularOffset *= ShooterInformation.ShooterConstants.BLUE_TURRET_ANGULAR_OFFSET_DIRECTION;
        else turretAngularOffset *= ShooterInformation.ShooterConstants.RED_TURRET_ANGULAR_OFFSET_DIRECTION;

        turretStartPosition = turret.startPosition; //turret.getCurrentPosition();
        turretPosition = turretStartPosition;

        if (goalCoordinates == Goal.GoalCoordinates.BLUE) relocalization(ShooterInformation.Odometry.RELOCALIZATION_POSES.BLUE_FAR_START_POSITION);
        else relocalization(ShooterInformation.Odometry.RELOCALIZATION_POSES.RED_FAR_START_POSITION);

        flywheel.reset();
    }

    private boolean shooterToggle = false;

    private double hoodPosition = 0.4;

    private double turretPosition;

    private double robotYawRad;
    public double tt;

    public Pose2d robotPose;

    private boolean automaticHoodToggle = true;
    private double distanceToGoal;

    public void update() {

        //getting robot pose
        if (controller2.main_buttonHasJustBeenPressed) relocalization(ShooterInformation.Odometry.RELOCALIZATION_POSES.BACK);
        robotYawRad = rev9AxisImuWrapped.getYaw(AngleUnit.RADIANS);
        customDrive.updatePoseEstimate();

        //turret
        double turretCurrentPosition = turret.getCurrentPosition(); //used to calculate turret pose

        if (controller2.dpad_leftHasJustBeenPressed) {
            turretStartPosition+=ShooterInformation.ShooterConstants.TURRET_HOME_POSITION_INCREMENT;
        }
        else if (controller2.dpad_rightHasJustBeenPressed) {
                turretStartPosition-=ShooterInformation.ShooterConstants.TURRET_HOME_POSITION_INCREMENT;
        }

        robotPose = ShooterInformation.Calculator.getBotPose(customDrive.localizer.getPose().position, robotYawRad);
        Pose2d turretPose = ShooterInformation.Calculator.getTurretPoseFromBotPose(robotPose.position, robotYawRad, turretCurrentPosition, turretStartPosition);

        Goal.GoalCoordinate goalCoordinate;

        //changing the coordinate that the turret aims at based on targeted zones determined by distance
        if (robotPose.position.x > ShooterInformation.ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER) {
            goalCoordinate = goalCoordinates.getCloseCoordinate();
        }
        else {
            goalCoordinate = goalCoordinates.getFarCoordinate();
        }

        double angleToGoal = Goal.getAngleToGoal(turretPose.position.x, turretPose.position.y, goalCoordinate);
        double rawtt = (angleToGoal - Math.toDegrees(robotYawRad) + turretAngularOffset);
        tt = route(rawtt);

        turretPosition = tt * ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE + turretStartPosition;
        double targetPosition = MathUtil.deadband(turretPosition, turret.getTargetPosition(), ShooterInformation.ShooterConstants.TURRET_DEADBAND_TICKS);

        if (controller2.left_trigger(Constants.TRIGGER_THRESHOLD)) turret.setPosition(turretStartPosition);
        else turret.setPosition(targetPosition);

        //flywheel
        if (controller1.left_bumperHasJustBeenPressed) shooterToggle = !shooterToggle;

        // setting flywheel velocity
        if (controller1.yHasJustBeenPressed) { //close

            flywheelTargetVelocityZone = ZONE.CLOSE;
            controller2.rumble(ShooterInformation.ShooterConstants.NORMAL_CONTROLLER_RUMBLE_TIME);
        }
        else if (controller1.bHasJustBeenPressed) { //far

            flywheelTargetVelocityZone = ZONE.FAR;
            controller2.rumble(ShooterInformation.ShooterConstants.NORMAL_CONTROLLER_RUMBLE_TIME);
        }

        if (shooterToggle) flywheel.setVelocity(getFlywheelTargetVelocity(), true);
        else flywheel.setVelocity(0, true);

        if (controller2.right_trigger(Constants.TRIGGER_THRESHOLD)) {
            if (flywheel.getMotorEnabled()) flywheel.runMotor(ExtremePrecisionFlywheel.RunningMotor.DISABLE);
            flywheel.setPower(1);
        }
        else if (!flywheel.getMotorEnabled()) {
            flywheel.runMotor(ExtremePrecisionFlywheel.RunningMotor.ENABLE);
        }

        // if turret is within an acceptable amount of error, the controller is rumbled.
        if (turret.getPositionError() < ShooterInformation.ShooterConstants.TURRET_TARGET_POSITION_ERROR_MARGIN) {
            controller1.rumble(ShooterInformation.ShooterConstants.NORMAL_CONTROLLER_RUMBLE_TIME);
        }
        else {
            controller1.stopRumble();
        }

        //hood
        if (controller2.shareHasJustBeenPressed) automaticHoodToggle = !automaticHoodToggle;

        if (flywheelTargetVelocityZone == ZONE.FAR) hoodPosition = ShooterInformation.ShooterConstants.HOOD_FAR_POSITION;
        else if (automaticHoodToggle) {

            Goal.GoalCoordinatesForDistance goalCoordinatesForDistance =
                    goalCoordinates == Goal.GoalCoordinates.BLUE
                    ? Goal.GoalCoordinatesForDistance.BLUE
                    : Goal.GoalCoordinatesForDistance.RED;

            Vector2d turretVector = turretPose.position;

            distanceToGoal = Goal.getDistanceFromGoal(turretVector.x, turretVector.y, goalCoordinatesForDistance.getCoordinate());

            hoodPosition = ShooterInformation.Models.getCloseHoodPositionFromRegression(distanceToGoal);
        }
        else noAutomaticHood();

        hoodAngler.setPosition(MathUtil.clamp(hoodPosition, ShooterInformation.ShooterConstants.HOOD_ANGLER_MAX_POSITION, ShooterInformation.ShooterConstants.HOOD_ANGLER_MIN_POSITION));

        //updating
        turret.update();
        flywheel.updateKvBasedOnVoltage();
        flywheel.update();
//        if (timer.milliseconds() >= Constants.FLYWHEEL_PIDFVAS_LOOP_TIME) {
//            //run instance of flywheel and turret systems
//            timer.reset();
//            flywheel.update();
//        }

    }

    private double getFlywheelTargetVelocity() {

        double flywheelTargetVelocity;

        if (flywheelTargetVelocityZone == ZONE.FAR) {
            flywheelTargetVelocity = ShooterInformation.ShooterConstants.FAR_SIDE_FLYWHEEL_SHOOT_VELOCITY;
        }
        else if (distanceToGoal < ShooterInformation.ShooterConstants.CLOSE_SIDE_SWITCH || !automaticHoodToggle) { //uses this close velocity if automatic hood isn't being used
            flywheelTargetVelocity = ShooterInformation.ShooterConstants.CLOSER_CLOSE_SIDE_FLYWHEEL_SHOOT_VELOCITY;
        }
        else {
            flywheelTargetVelocity = ShooterInformation.ShooterConstants.FARTHER_CLOSE_SIDE_FLYWHEEL_SHOOT_VELOCITY;
        }
        return flywheelTargetVelocity;
    }

    private void noAutomaticHood() {

        if (!(controller2.yHasJustBeenPressed || controller2.xHasJustBeenPressed || controller2.bHasJustBeenPressed || controller2.aHasJustBeenPressed)) {
            return;
        }

        if (controller2.yHasJustBeenPressed) { //close
            ShooterInformation.ShooterConstants.HOOD_CLOSE_POSITION+=ShooterInformation.ShooterConstants.HOOD_POSITION_MANUAL_INCREMENT;
        }
        else if (controller2.xHasJustBeenPressed) {
            ShooterInformation.ShooterConstants.HOOD_CLOSE_POSITION-=ShooterInformation.ShooterConstants.HOOD_POSITION_MANUAL_INCREMENT;
        }

        if (controller2.bHasJustBeenPressed) { //far
            hoodPosition = ShooterInformation.ShooterConstants.HOOD_FAR_POSITION+=ShooterInformation.ShooterConstants.HOOD_POSITION_MANUAL_INCREMENT;
        }
        else if (controller2.aHasJustBeenPressed) {
            ShooterInformation.ShooterConstants.HOOD_FAR_POSITION-=ShooterInformation.ShooterConstants.HOOD_POSITION_MANUAL_INCREMENT;
        }

        manualUpdateHoodPositions();
    }

    private void manualUpdateHoodPositions() {

        if (flywheelTargetVelocityZone == ZONE.CLOSE) {
            hoodPosition = ShooterInformation.ShooterConstants.HOOD_CLOSE_POSITION;
        }
        else if (flywheelTargetVelocityZone == ZONE.FAR) {
            hoodPosition = ShooterInformation.ShooterConstants.HOOD_FAR_POSITION;
        }
    }

    private double route(double rawtt) {

        if (rawtt >= ShooterInformation.ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES && rawtt <= ShooterInformation.ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES) return rawtt; //no need to reroute

        double[] reroutes = {rawtt - 360, rawtt + 360};
        if (reroutes[0] >= ShooterInformation.ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES && reroutes[0] <= ShooterInformation.ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES) {
            return reroutes[0];
        }
        else if (reroutes[1] >= ShooterInformation.ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES && reroutes[1] <= ShooterInformation.ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES) {
            return reroutes[1];
        }

        // go to closest limit if target position is outside the min and max
        else if (rawtt < ShooterInformation.ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES) return ShooterInformation.ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES;
        else return ShooterInformation.ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES;
    }

    private void relocalization(ShooterInformation.Odometry.RELOCALIZATION_POSES pose) {

        double heading = ShooterInformation.Odometry.REZERO_POSES[pose.getPoseIndex()][2];

        Pose2d reZeroPose = new Pose2d(

                ShooterInformation.Odometry.REZERO_POSES[pose.getPoseIndex()][0],
                ShooterInformation.Odometry.REZERO_POSES[pose.getPoseIndex()][1],
                Math.toRadians(heading)
        );

        customDrive.localizer.setPose(reZeroPose);
        rev9AxisImuWrapped.setYaw(heading);
    }

    public double rev9AxisImuHeadingDeg() {
        return Math.toDegrees(robotYawRad);
    }

}
