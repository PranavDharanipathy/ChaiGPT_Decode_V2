package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class PPConstants {

    public static FollowerConstants followerConstants = new FollowerConstants()

            .mass(13.19954)

            .forwardZeroPowerAcceleration(66.62530181163879)
            .lateralZeroPowerAcceleration(-51.82186721441313)

            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(false)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.13, 0, 0.018, 0.175))

            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.055,0.1))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0,0.0018,0.85,0.1))

            .translationalPIDFSwitch(3)
            .headingPIDFSwitch(0.17453299)
            .drivePIDFSwitch(20)

            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.003,0.035))

            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.1,0.0005,0,0.025))

            .centripetalScaling(0.00062)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.994, 50, 1.5, 1.5);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("right_front")
            .rightRearMotorName("right_back")
            .leftRearMotorName("left_back")
            .leftFrontMotorName("left_front")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            .xVelocity(-41.2745147944275)
            .yVelocity(-61.15854729822397);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.75)
            .strafePodX(8.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
