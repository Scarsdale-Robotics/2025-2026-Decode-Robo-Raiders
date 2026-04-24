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
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.TravelCommand
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.intakePower
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.maxPower
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.ShootCommand
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.finalGoShoot
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.robotGateIntake
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.robotGateIntakeOneShot
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.robotGoShoot
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.robotIntake
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.robotShoot
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonUtil.parkRobot

@Autonomous(name = "[C-COOP-18-B] Auton Blue Close CoOp", group = "Auton")
class AutonBlueCloseCoOp : AutonBase(
    true,
    3.5,
    144.0 - 3.5,
    AutonPositions.Blue(AutonPositions.startPoseClose),
    { isBlue, follower -> {
        val pb = { follower.pathBuilder() }

        val DOWN = Ang(270.deg.inRad, isBlue)
        val LEFT = Ang(180.deg.inRad, isBlue)
        val DOWN_LEFT = Ang(225.deg.inRad, isBlue)
        val GATE_INITIAL = Ang(150.deg.inRad, isBlue)
        val GATE = Ang(110.deg.inRad, isBlue)

        val FRIED = Ang(165.deg.inRad, isBlue)

        val xIntakeThreshold = 51.0
        val gateXIntakeThreshold = 35.0

        val startPose = Pos(AutonPositions.startPoseClose, isBlue)

        val preloadShootPose = Pos(Pose(51.0+2.32, 92.0+3.0), isBlue)
        val preloadPath = pb().addPath(BezierLine(startPose, preloadShootPose))
            .setLinearHeadingInterpolation(DOWN, LEFT, 0.3)
            .setTimeoutConstraint(0.0)
            .build()

        val setLine2Pose = Pos(Pose(13.7+2.32, 57.0+3.0), isBlue) //19.5, 60.0
        val setLine2Ctrl = Pos(Pose(52.9+2.32, 55.6+3.0), isBlue) //48.9, 59.2

        val setLine2Path = pb().addPath(BezierCurve(preloadShootPose, setLine2Ctrl, setLine2Pose))
            .setConstantHeadingInterpolation(LEFT)
//            .setLinearHeadingInterpolation(LEFT, FRIED, 1.0, 0.75)
            .addCallback({ X(follower.pose.x, isBlue) < xIntakeThreshold }, IntakeCommand)
//            .addCallback({ X(follower.pose.x, isBlue) < xIntakeThreshold - 4.0 }, intakePower)
            .setTimeoutConstraint(0.0)
            .build()

        val experi_L2 = Pos(Pose(19.1+2.32, 61.0+3.0), isBlue)
        val experi_L2Ctrl = Pos(Pose(24.8+2.32, 57.0+3.0), isBlue)
        val experiPathL2 = pb().addPath(BezierCurve(setLine2Pose, experi_L2Ctrl, experi_L2))
            .setConstantHeadingInterpolation(LEFT)
//            .setLinearHeadingInterpolation(LEFT, FRIED, 1.0, 0.75)
//            .addCallback({ X(follower.pose.x, isBlue) < xIntakeThreshold }, IntakeCommand)
//            .addCallback({ X(follower.pose.x, isBlue) < xIntakeThreshold - 4.0 }, intakePower)
            .setTimeoutConstraint(0.0)
            .build()


        val shoot2Pose = Pos(Pose(61.0+2.32, 71.0+3.0), isBlue)
        val shoot2Path = pb().addPath(BezierLine(experi_L2, shoot2Pose)) //was setLine2
//            .addParametricCallback(0.0, maxPower)
            .setConstantHeadingInterpolation(LEFT)
            .setTimeoutConstraint(0.0)
            .build()

        val gateIntakePose = Pos(Pose(15.25, 57.2), isBlue)
        val gateIntakePath = pb().addPath(BezierLine(shoot2Pose, gateIntakePose))
            .setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                    HeadingInterpolator.PiecewiseNode(
                        0.0, 0.5, HeadingInterpolator.constant(LEFT)
                    ),
                    HeadingInterpolator.PiecewiseNode(
                        0.5, 0.8, HeadingInterpolator.linear(LEFT, GATE_INITIAL)
                    ),
                    HeadingInterpolator.PiecewiseNode(
                        0.8, 1.0, HeadingInterpolator.constant(GATE_INITIAL)
                    ),
                )
            )
            .addCallback({ X(follower.pose.x, isBlue) < gateXIntakeThreshold }, IntakeCommand)
            .setTimeoutConstraint(0.0)
            .build()

        val gateIntakeBackupPose = Pos(Pose(15.0, 54.2), isBlue)
        val gateIntakeBackupPath = pb().addPath(BezierLine(gateIntakePose, gateIntakeBackupPose))
            .setLinearHeadingInterpolation(GATE_INITIAL, GATE)
            .setTimeoutConstraint(0.0)
            .build()

        val shootGatePose = shoot2Pose
        val shootGatePath = pb().addPath(BezierLine(gateIntakeBackupPose, shootGatePose))
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

        val setLine1Pose = Pos(Pose(22.5+2.32, 87.0+3.0), isBlue)
        val setLine1Path = pb().addPath(BezierLine(shootGatePose, setLine1Pose))
            .setConstantHeadingInterpolation(LEFT)
            .addCallback({ X(follower.pose.x, isBlue) < xIntakeThreshold }, IntakeCommand)
            .setTimeoutConstraint(0.0)
            .build()

        val shoot1Pose = Pos(Pose(56.7+2.32, 97.0+3.0), isBlue)
        val shoot1Path = pb().addPath(BezierLine(setLine1Pose, shoot1Pose))
            .setConstantHeadingInterpolation(DOWN_LEFT)
            .setTimeoutConstraint(0.0)
            .build()

        val parkPath = pb().addPath(BezierLine(shoot2Pose, AutonPositions.autonParkPose))
            .setConstantHeadingInterpolation(DOWN_LEFT)
            .setTimeoutConstraint(0.0)
            .build()

        SequentialGroup(
            // 3
            ParallelGroup(
                FollowPath(preloadPath),
                SequentialGroup(
                    Delay(1.4),
                    ShootCommand
                )
            ),

            // 6
            robotIntake(setLine2Path),
            FollowPath(experiPathL2),

            robotGoShoot(shoot2Path),
            robotShoot(),

            // 9
            robotGateIntake(gateIntakePath, gateIntakeBackupPath),
            robotGoShoot(shootGatePath),
            robotShoot(),

            // 12
            robotGateIntake(gateIntakePath, gateIntakeBackupPath),
            robotGoShoot(shootGatePath),
            robotShoot(),

            // 15
            robotGateIntake(gateIntakePath, gateIntakeBackupPath),
            robotGoShoot(shootGatePath),
            robotShoot(),

//            // 18
//            robotGateIntake(gateIntakePath, gateIntakeBackupPath),
//            robotGoShoot(shootGatePath),
//            robotShoot(),

            // 21
            robotIntake(setLine1Path),
            finalGoShoot(shoot1Path),

            ShootCommand,
//            parkRobot(parkPath)

        )
    }() }
)
