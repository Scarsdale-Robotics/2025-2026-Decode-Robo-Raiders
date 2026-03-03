package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
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
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(28.9) //in kg(MORE LIKELY TO BE IN POUNDS)
            .forwardZeroPowerAcceleration(-29.926218219482156)
            .lateralZeroPowerAcceleration(-61.48754053052945)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.1,
                    0,
                    0.01,
                    0.02
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.12,
                    0,
                    0.003,
                    0.01
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.5,
                    0,
                    0.12,
                    0.02
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2.5,
                    0,
                    0.1,
                    0.0003
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.015,
                    0,
                    0.00012, //0.00035
                    0, //0.6
                    0.012 //0.015
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.012,
                    0,
                    0.00001,
                    0.0,
                    0.0
            ))
            .drivePIDFSwitch(15)

            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)

            .centripetalScaling(0.00028);




    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(79.5032776359498)
            .yVelocity(61.580920121801185)
            .rightFrontMotorName("rfw")
            .rightRearMotorName("rbw")
            .leftRearMotorName("lbw")
            .leftFrontMotorName("lfw")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-95.64271)
            .strafePodX(-24.64271)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.8, 0.85);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
