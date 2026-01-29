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
        .mass(25.2) //in kg
            .forwardZeroPowerAcceleration(-36.251943126849014)
            .lateralZeroPowerAcceleration(-69.11570394121402)
            .translationalPIDFCoefficients(new PIDFCoefficients( //fine
                    0.1,
                    0,
                    0.01,
                    0.02
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients( //fine
                    0.1,
                    0,
                    0.007,
                    0.009
            ))
            .headingPIDFCoefficients(new PIDFCoefficients( //fine
                    1.2,
                    0,
                    0.1,
                    0.02
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients( //fine
                    1.5,
                    0,
                    0.12,
                    0.001
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients( //fine
                    0.01460,
                    0,
                    0.00012, //0.00035
                    0, //0.6
                    0.02 //0.015
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients( //fine ig
                    0.032,
                    0,
                    0.00004,
                    0.0,
                    0.0
            ))
            .drivePIDFSwitch(15)

            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)

            .centripetalScaling(0.0002);




    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(65.31741164800691)
            .yVelocity(58.67722927303764)
            .rightFrontMotorName("rfw")
            .rightRearMotorName("rbw")
            .leftRearMotorName("lbw")
            .leftFrontMotorName("lfw")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-96)
            .strafePodX(-25)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1
            , 1);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
