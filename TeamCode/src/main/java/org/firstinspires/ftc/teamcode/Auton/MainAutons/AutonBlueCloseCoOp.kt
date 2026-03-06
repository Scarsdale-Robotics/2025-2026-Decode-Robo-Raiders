package org.firstinspires.ftc.teamcode.Auton.MainAutons
import org.firstinspires.ftc.teamcode.Auton.AutonPositions

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonBlueCloseArtifact24.Companion.intakeSpeed

import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.lower.MagServoSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.lower.LowerMotorSubsystem

import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.BORD_Y
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distAndVeloToThetaClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distAndVeloToThetaFar
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToTimeClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToTimeFar
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToVelocityClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToVelocityFar
import org.firstinspires.ftc.teamcode.utils.Lefile
import java.io.File
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min

@Autonomous(name = "[COOP-18] Auton Blue Close CoOp", group = "Auton")
@Configurable
class AutonBlueCloseCoOp: NextFTCOpMode() {
    //////////////////////
    ////Base Variables////
    //////////////////////
    private var pathTimer: Timer? = null
    var actionTimer: Timer? = null;
    var opmodeTimer: Timer? = null;
    init { addComponents(
        PedroComponent(Constants::createFollower),
        SubsystemComponent(
            LowerSubsystem,
            OuttakeSubsystem
        )
    );
        actionTimer = Timer()
        pathTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer!!.resetTimer()
    }

    companion object {
        val delayStartShoot: Double = 0.4
        val DelayBeforeShoot: Double = 0.12
        val delayAfterEachShoot: Double = 0.35 //currently at a really high #
        val DelayForParterBot: Double = 0.4
        val DelayInIntake: Double = 0.7
        val delayAtGate: Double = 0.2

        val goalX = 3.0
        val goalY = 144.0 - 6.0

        val intakeSpeed = 0.5
//        var directionGoalX = 4.0;
//        var directionGoalY = 144.0-4.0;
    }

    /////////////////
    ////Positions////
    /////////////////
    //Constant positions
//    private val shootPoseCloseFromIntake1 = Pose(57.0, 76.6, Math.toRadians(128.0)) // Close Shoot Pose of our robot.


    /////////////
    ////Paths////
    /////////////
    // The different paths the robot will take in during Auton
    private var robotShootPreload: PathChain? = null

    private var robotIntake1: PathChain? = null
    private var robotGoToShoot1: PathChain? = null //since preload also intakes, there is no intake1

    private var robotIntake2: PathChain? = null
    private var robotGoToShoot2: PathChain? = null

    private var robotFirstLeverOpen: PathChain? = null //very important

    private var robotCommonIntake: PathChain? = null
    private var robotCommonGoShoot: PathChain? = null

    private var robotPark: PathChain? = null

    ////////////////////
    ////Path Builder////
    ////////////////////
    fun buildPaths() {
        //Shoots preload and intakes once
        robotShootPreload = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.startPoseClose),
                    AutonPositions.Blue(AutonPositions.start24ShootPos),
                )
            )
//            .setTangentHeadingInterpolation()
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.start24ShootPos).heading)
            .build()

        robotIntake1 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.start24ShootPos),
                    AutonPositions.Blue(AutonPositions.intake1Pos24)
                )
            )
//            .setTangentHeadingInterpolation()
            .addParametricCallback(0.90, IntakeCommand) //WHERE INTAKE COMMAND WILL NOW GO IG
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.intake1Pos24).heading)
            .build()

        robotFirstLeverOpen = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.intake1Pos24),
                    AutonPositions.Blue(AutonPositions.coOpFirstGateOpenControlPos),
                    AutonPositions.Blue(AutonPositions.coOpFirstStartGateOpen),
                )
            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.coOpFirstStartGateOpen).heading)
            .build()

        //1st Go Shoot
        robotGoToShoot1 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.coOpFirstStartGateOpen),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.coOpFirstStartGateOpen).heading)
            .build()

        //2nd Intake
        robotIntake2 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.coOpStartGateOpenControlPos),
                    AutonPositions.Blue(AutonPositions.coOpStartGateOpen)
                )
            )
            .addParametricCallback(0.90, IntakeCommand) //WHERE INTAKE COMMAND WILL NOW GO IG
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.shootPoseClose).heading,
                AutonPositions.Blue(AutonPositions.coOpStartGateOpen).heading,
                0.5)
            .build()

        //2nd Go Shoot
        robotGoToShoot2 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.coOpStartGateOpen),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.coOpStartGateOpen).heading,
                AutonPositions.Blue(AutonPositions.shootPoseClose).heading,
                0.9)
            .build()

        //3rd Intake
        robotCommonIntake = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.CoOpCommonIntakeControlPos1),
                    AutonPositions.Blue(AutonPositions.CoOpCommonIntakeControlPos2),
                    AutonPositions.Blue(AutonPositions.CoOpCommonIntake)
                )
            )
            .addParametricCallback(0.90, IntakeCommand) //WHERE INTAKE COMMAND WILL NOW GO IG
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.CoOpCommonIntake).heading)
            .build()

        //3rd Go Shoot
        robotCommonGoShoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.CoOpCommonIntake),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
            .setLinearHeadingInterpolation(AutonPositions.Blue(AutonPositions.CoOpCommonIntake).heading, AutonPositions.Blue(AutonPositions.shootPoseClose).heading, 0.9)
            .build()

        //Go Park
        robotPark = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.autonParkPose)
                )
            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.autonParkPose).heading)
            .build()
    }

    ///////////////////////////
    ////Main Auton Commands////
    ///////////////////////////
    val intakePower: Command = InstantCommand {PedroComponent.follower.setMaxPower(intakeSpeed)}
    val maxPower: Command = InstantCommand {PedroComponent.follower.setMaxPower(1.0)}

    val IntakeCommand: Command
        get() = ParallelGroup(
            intakePower,
            IntakeMotorSubsystem.intake,
            MagMotorSubsystem.intake,
//            MagServoSubsystem.run,
            MagblockServoSubsystem.block

        )
    val IntakeAfterCommand: Command
        get() = ParallelGroup(
            maxPower,
            IntakeMotorSubsystem.intake,
            MagMotorSubsystem.intake,
//            MagServoSubsystem.run,
            MagblockServoSubsystem.block

        )
    val TravelCommand: Command
        get() = ParallelGroup(
            IntakeMotorSubsystem.off,
            MagMotorSubsystem.off,
//            MagServoSubsystem.stop,
            MagblockServoSubsystem.block
        )
    val ShootCommand: Command
        get() = ParallelGroup(
            MagblockServoSubsystem.unblock,
            MagMotorSubsystem.On(1.0),
            IntakeMotorSubsystem.intake,
//            MagServoSubsystem.run
        )

    /////////////////////////
    ////Autonomous Runner////
    /////////////////////////
    private val autonomousRoutine: Command
        get() = SequentialGroup(
            //Main Group
            SequentialGroup(
                FollowPath(robotShootPreload!!),
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                TravelCommand,
                FollowPath(robotIntake1!!),

                ParallelGroup( //Robot goes back to CLOSE Shoot Position
                    SequentialGroup(
                        IntakeAfterCommand,
                        Delay(DelayInIntake),
                        TravelCommand
                    ),
                    FollowPath(robotFirstLeverOpen!!),
                ),
                Delay(delayAtGate),
            ),

            ParallelGroup( //Robot goes back to CLOSE Shoot Position
//                TurretPhiSubsystem.SetTargetPhi((- (-5.075 + 2.0 * PI - PI / 3.0 - PI / 48.0)).rad),
                SequentialGroup(
                    IntakeAfterCommand,
                    Delay(DelayInIntake),
                ),
                FollowPath(robotGoToShoot1!!)
            ),

            SequentialGroup( //Shoots FIRST Intake, goes to intake and open lever
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                TravelCommand,
                FollowPath(robotIntake2!!), //robot goes to intake
            ),

            ParallelGroup( //Robot goes back to CLOSE Shoot Position
                SequentialGroup(
                    IntakeAfterCommand,
                    Delay(DelayInIntake),
                ),
                FollowPath(robotGoToShoot2!!)
            ),

            SequentialGroup( //Shoots SECOND Intake, goes to intake and then OPEN lever
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                TravelCommand,
                FollowPath(robotIntake2!!), //robot goes to intake
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
                SequentialGroup(
                    IntakeAfterCommand,
                    Delay(DelayInIntake),
                ),
                FollowPath(robotGoToShoot2!!)
            ),

            //////////////////////////////////
            ////////REPEATABLE SECTION////////
            //////////////////////////////////
            SequentialGroup( //Shoots THIRD Intake, goes to OPEN lever and then intake
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                TravelCommand,
                FollowPath(robotCommonIntake!!), //robot goes to intake
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
                SequentialGroup(
                    Delay(DelayInIntake),
                    IntakeAfterCommand,
                ),
                FollowPath(robotCommonGoShoot!!)
            ),

            SequentialGroup( //Shoots THIRD Intake, goes to OPEN lever and then intake
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                TravelCommand,
                Delay(DelayForParterBot), //Temporary Pos for this delay until figure out how needed or changed delay has to be
                FollowPath(robotCommonIntake!!), //robot goes to intake
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
                SequentialGroup(
                    IntakeAfterCommand,
                    Delay(DelayInIntake),
                ),
                FollowPath(robotCommonGoShoot!!)
            ),
            //////////////////////////////////
            ////////REPEATABLE SECTION////////
            //////////////////////////////////

            SequentialGroup( //Shoots SIXTH Intake, goes to PARK
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                TravelCommand,
                InstantCommand { stopShooterAutoAim = true },
                ShooterSubsystem.On(9999.0),
                FollowPath(robotPark!!), //robot goes to intake
            ),
        )

    private var stopShooterAutoAim = false;
    override fun onUpdate() {
        val dx = AutonBlueFarArtifact.Companion.goalX - PedroComponent.follower.pose.x
        val dy = AutonBlueFarArtifact.Companion.goalY - PedroComponent.follower.pose.y
        val dxy = hypot(dx, dy)
        val dxp = dx - PedroComponent.follower.velocity.xComponent * (
                if (PedroComponent.follower.pose.y < BORD_Y) distanceToTimeFar(dxy)
                else distanceToTimeClose(dxy)
                )
        val dyp = dy - PedroComponent.follower.velocity.yComponent * (
                if (PedroComponent.follower.pose.y < BORD_Y) distanceToTimeFar(dxy)
                else distanceToTimeClose(dxy)
                )
        val dxyp = hypot(dxp, dyp)

        if (!stopShooterAutoAim) {
            ShooterSubsystem.AutoAim(
                dxy,
                { dist ->
                    (
                            if (PedroComponent.follower.pose.y < BORD_Y)
                                distanceToVelocityFar(dist)
                            else
                                distanceToVelocityClose(dist)
                            )
                }
            )()
        }
        TurretPhiSubsystem.AutoAim(dxp, dyp, PedroComponent.follower.pose.heading.rad)();
        TurretThetaSubsystem.AutoAim(
            dxyp,
            { dist ->
                if (PedroComponent.follower.pose.y < BORD_Y)
                    distAndVeloToThetaFar(dist, ShooterSubsystem.velocity)
                else
                    distAndVeloToThetaClose(dist, ShooterSubsystem.velocity)
            },
        )()

        // These loop the movements of the robot, these must be called continuously in order to work
//        follower!!.update();
//        forward.schedule()
        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState)

        telemetry.addData("x", PedroComponent.follower.pose.x)
        telemetry.addData("y", PedroComponent.follower.pose.y)
        telemetry.addData("heading", PedroComponent.follower.pose.heading)
        telemetry.update()

        PanelsTelemetry.telemetry.update()
    }

    /** This method is called continuously after Init while waiting for "play". **/
    override fun onInit() {
//        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        ShooterSubsystem.off()
        IntakeMotorSubsystem.off()
        MagMotorSubsystem.off()
//        MagServoSubsystem.stop()
        MagblockServoSubsystem.block()

        PedroComponent.follower.setStartingPose(AutonPositions.Blue(AutonPositions.startPoseClose))
    }

    /** This method is called continuously after Init while waiting for "play".  */
    override fun onWaitForStart() {}

    fun distanceToTime(it: Double): Double {
        return it * 0.0;
    }
    fun distanceToVelocity(it: Double): Double {
        return 0.0127 * it * it + 1.81 * it + 937.0;
    }
    fun distanceToTheta(it: Double): Angle {
        return max(min(-0.224*it+74, 63.0), 55.0).deg;
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system  */
    override fun onStartButtonPressed() {
        autonomousRoutine()

        opmodeTimer!!.resetTimer()
        actionTimer!!.resetTimer()
//        setPathState(AutonPath.RobotShoot1)
    }

    /** We do not use this because everything should automatically disable  */
    override fun onStop() {
        val file = File(Lefile.filePath)
        file.writeText(
            PedroComponent.follower.pose.x.toString() + "\n" +
                    PedroComponent.follower.pose.y.toString() + "\n" +
                    PedroComponent.follower.pose.heading.toString() + "\n"
        )
    }
}