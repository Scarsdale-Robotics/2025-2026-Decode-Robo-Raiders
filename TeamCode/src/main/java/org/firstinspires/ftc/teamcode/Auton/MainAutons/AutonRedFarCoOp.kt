package org.firstinspires.ftc.teamcode.Auton.MainAutons

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.units.deg
import org.firstinspires.ftc.teamcode.Auton.AutonPositions
import org.firstinspires.ftc.teamcode.Auton.AutonPositions.Ang
import org.firstinspires.ftc.teamcode.Auton.AutonPositions.Pos
import org.firstinspires.ftc.teamcode.Auton.AutonPositions.X
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.IntakeCommand
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.robotGoShoot
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.robotIntake
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.robotShoot
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.parkRobot

import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem

@Autonomous(name = "[F-COOP-18-R] Auton Red Far CoOp", group = "Auton")
class AutonRedFarCoOp : AutonBase(
    false,
    144.0 - 3.5,
    144.0 - 3.5,
    AutonPositions.Red(AutonPositions.startPose),
    { isBlue, follower -> {
//        var Portal = CvBallDetectionP(hardwareMap)
        val pb = { follower.pathBuilder() }

        val DOWN = Ang(270.deg.inRad, isBlue)
        val LEFT = Ang(180.deg.inRad, isBlue)

        val xIntakeThreshold = 50.0
        val xCommonThreshold = 45.5

        val startPose = Pos(AutonPositions.startPose, isBlue)

        val setLine3Pose = Pos(Pose(13.3, 35.9), isBlue)
        val setLine3Ctrl = Pos(Pose(40.0, 41.1), isBlue)
        val setLine3Path = pb().addPath(BezierCurve(startPose, setLine3Ctrl, setLine3Pose))
            .setConstantHeadingInterpolation(LEFT)
//            .setLinearHeadingInterpolation(LEFT, FRIED, 1.0, 0.75)
            .addCallback({ X(follower.pose.x, isBlue) < xIntakeThreshold }, IntakeCommand)
            .setTimeoutConstraint(0.0)
            .build()

        val shoot1Pose = Pos(Pose(53.9, 12.7), isBlue)
        val shoot1Path = pb().addPath(BezierLine(setLine3Pose, shoot1Pose))
//            .setConstantHeadingInterpolation(DOWN_LEFT)
            .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
            .setTimeoutConstraint(0.0)
            .build()

        val setCommonPose = Pos(Pose(13.4, 11.0), isBlue)
        val setCommonPath = pb().addPath(BezierLine(shoot1Pose, setCommonPose))
            .setConstantHeadingInterpolation(LEFT)
            .addCallback({ X(follower.pose.x, isBlue) < xCommonThreshold }, IntakeCommand)
            .setTimeoutConstraint(0.0)
            .build()

        val setCommonShootPath = pb().addPath(BezierLine(setCommonPose, shoot1Pose))
//            .setConstantHeadingInterpolation(DOWN_LEFT)
            .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
            .setTimeoutConstraint(0.0)
            .build()

//        fun getBlobPath(): PathChain {
////            var Portal: CvBallDetectionP? = null
//            var Blobs: MutableList<ColorBlobLocatorProcessor.Blob>? = null
//            var cd = 0.0
//
//            var min = Double.Companion.MAX_VALUE // reset each loop iteration
//            var Minb: ColorBlobLocatorProcessor.Blob? = null
//            var rad = -1.0
//
//            Portal!!.updateDetections()
//            Blobs = Portal.getBlobs()
//
//            if (Blobs != null && !Blobs.isEmpty()) {
//                if (Blobs.isEmpty()) {
//                    cd = -1.0
//                }
//                for (b in Blobs) {
//                    val circleFit = b.getCircle()
//
//                    if (circleFit == null) continue
//
//                    val radius = circleFit.getRadius().toDouble()
//
//                    if (radius == 0.0) continue
//                    cd = ((120.0 * 529) / (radius * 2)) * 2
//                    val theta = Math.toDegrees(atan2((circleFit.getX() - 320).toDouble(), 391.0))
//
//                    if (cd < min) {
//                        min = cd
//                        rad = theta
//                        Minb = b
//                    }
//                }
//            }
//
//            var intakePos: Pose
//
//            if (min != -1.0) {
//                val xa: Double = Math.sin(rad) * min
//                val ya: Double = Math.cos(rad) * min
//                intakePos = Pose(shoot1Pose.x + xa, shoot1Pose.y + ya)
//
//                return PedroComponent.follower.pathBuilder()
//                    .addPath(
//                        BezierLine(
//                            shoot1Pose,
//                            intakePos
//                        )
//                    )
//                    .setTangentHeadingInterpolation()
//                    .addCallback({ X(follower.pose.x, isBlue) < xCommonThreshold }, IntakeCommand)
//                    .build()
//            }
//            Portal.close();
//            return setCommonPath!!
//        }

        val setParkPose = Pos(Pose(44.4, 15.3), isBlue)
        val parkPath = pb().addPath(BezierLine(shoot1Pose, setParkPose))
//            .setConstantHeadingInterpolation(DOWN_LEFT)
            .setTimeoutConstraint(0.0)
            .build()

        SequentialGroup(
            // 3
            SequentialGroup(
                MagblockServoSubsystem.unblock,
                Delay(1.5),
                robotShoot()
            ),

            // 6
            robotIntake(setLine3Path),
            robotGoShoot(shoot1Path),
            robotShoot(),

            // 9
            robotIntake(setCommonPath),
            Delay(0.2),
            robotGoShoot(setCommonShootPath),
            robotShoot(),

            // 12
            robotIntake(setCommonPath),
            Delay(0.2),
            robotGoShoot(setCommonShootPath),
            robotShoot(),

            // 15
            robotIntake(setCommonPath),
            Delay(0.2),
            robotGoShoot(setCommonShootPath),
            robotShoot(),

            // 18
            robotIntake(setCommonPath),
            Delay(0.2),
            robotGoShoot(setCommonShootPath),
            robotShoot(),

            // 21
//            robotIntake(setCommonPath),
//            Delay(0.2),
//            robotGoShoot(setCommonShootPath),
//            robotShoot(),

            parkRobot(parkPath)


        )
    }() }
)
