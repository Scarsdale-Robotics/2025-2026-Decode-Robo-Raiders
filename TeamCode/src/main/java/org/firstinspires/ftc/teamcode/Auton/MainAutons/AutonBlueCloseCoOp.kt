package org.firstinspires.ftc.teamcode.Auton.MainAutons

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathBuilder
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.control.interpolators.ConstantInterpolator
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.units.deg
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.Auton.AutonPositions
import org.firstinspires.ftc.teamcode.Auton.AutonPositions.Ang
import org.firstinspires.ftc.teamcode.Auton.AutonPositions.Pos
import org.firstinspires.ftc.teamcode.Auton.AutonPositions.X
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.IntakeCommand
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.ShootCommand
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.finalGoShoot
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.robotGateIntakeOneShot
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.robotGoShoot
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.robotIntake
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.robotShoot

@Autonomous(name = "[COOP-21-B] Auton Blue Close CoOp", group = "Auton")
class AutonBlueCloseCoOp : AutonBase(
    true,
    6.0,
    144.0 - 6.0,
    { isBlue, follower -> {
        val pb = follower.pathBuilder()

        val DOWN = Ang(270.deg.inRad, isBlue)
        val LEFT = Ang(180.deg.inRad, isBlue)
        val DOWN_LEFT = Ang(225.deg.inRad, isBlue)
        val GATE = Ang(145.deg.inRad, isBlue)

        val xIntakeThreshold = 50.0
        val gateXIntakeThreshold = 35.0

        val startPose = Pos(AutonPositions.startPoseClose, isBlue)

        val preloadShootPose = Pos(Pose(51.0, 92.0), isBlue)
        val preloadPath = pb.addPath(BezierLine(startPose, preloadShootPose))
            .setLinearHeadingInterpolation(DOWN, LEFT, 0.2)
            .setTimeoutConstraint(0.0)
            .build()

        val setLine2Pose = Pos(Pose(21.5, 64.0), isBlue)
        val setLine2Ctrl = Pos(Pose(45.0, 55.0), isBlue)
        val setLine2Path = pb.addPath(BezierCurve(preloadShootPose, setLine2Ctrl, setLine2Pose))
            .setConstantHeadingInterpolation(LEFT)
            .addCallback({ X(follower.pose.x, isBlue) < xIntakeThreshold }, IntakeCommand)
            .setTimeoutConstraint(0.0)
            .build()

        val shoot2Pose = Pos(Pose(61.0, 71.0), isBlue)
        val shoot2Path = pb.addPath(BezierLine(setLine2Pose, shoot2Pose))
            .setConstantHeadingInterpolation(LEFT)
            .setTimeoutConstraint(0.0)
            .build()

        val gateIntakePose = Pos(Pose(14.7266, 58.0), isBlue)
        val gateIntakePath = pb.addPath(BezierLine(shoot2Pose, gateIntakePose))
            .setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                    HeadingInterpolator.PiecewiseNode(
                        0.0, 0.5, HeadingInterpolator.constant(LEFT)
                    ),
                    HeadingInterpolator.PiecewiseNode(
                        0.5, 0.8, HeadingInterpolator.linear(LEFT, GATE)
                    ),
                    HeadingInterpolator.PiecewiseNode(
                        0.8, 1.0, HeadingInterpolator.constant(GATE)
                    ),
                )
            )
            .addCallback({ X(follower.pose.x, isBlue) < gateXIntakeThreshold }, IntakeCommand)
            .setTimeoutConstraint(0.0)
            .build()

        val shootGatePose = shoot2Pose
        val shootGatePath = pb.addPath(BezierLine(gateIntakePose, shootGatePose))
            .setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                    HeadingInterpolator.PiecewiseNode(
                        0.0, 0.3, HeadingInterpolator.linear(GATE, LEFT)
                    ),
                    HeadingInterpolator.PiecewiseNode(
                        0.3, 1.0, HeadingInterpolator.constant(LEFT)
                    ),
                )
            )
            .setTimeoutConstraint(0.0)
            .build()

        val setLine1Pose = Pos(Pose(21.5, 87.0), isBlue)
        val setLine1Path = pb.addPath(BezierLine(shootGatePose, setLine1Pose))
            .setConstantHeadingInterpolation(LEFT)
            .addCallback({ X(follower.pose.x, isBlue) < xIntakeThreshold }, IntakeCommand)
            .setTimeoutConstraint(0.0)
            .build()

        val shoot1Pose = Pos(Pose(55.0, 104.0), isBlue)
        val shoot1Path = pb.addPath(BezierLine(setLine1Pose, shoot1Pose))
            .setConstantHeadingInterpolation(DOWN_LEFT)
            .setTimeoutConstraint(0.0)
            .build()

        SequentialGroup(
            // 3
            ParallelGroup(
                FollowPath(preloadPath),
                SequentialGroup(
                    Delay(1.0),
                    ShootCommand
                )
            ),

            // 6
            robotIntake(setLine2Path),
            robotGoShoot(shoot2Path),
            robotShoot(),

            // 9
            robotGateIntakeOneShot(gateIntakePath),
            robotGoShoot(shootGatePath),
            robotShoot(),

            // 12
            robotGateIntakeOneShot(gateIntakePath),
            robotGoShoot(shootGatePath),
            robotShoot(),

            // 15
            robotGateIntakeOneShot(gateIntakePath),
            robotGoShoot(shootGatePath),
            robotShoot(),

            // 18
            robotGateIntakeOneShot(gateIntakePath),
            robotGoShoot(shootGatePath),
            robotShoot(),

            // 21
            robotIntake(setLine1Path),
            finalGoShoot(shoot1Path),

            ShootCommand
        )
    }() }
)
