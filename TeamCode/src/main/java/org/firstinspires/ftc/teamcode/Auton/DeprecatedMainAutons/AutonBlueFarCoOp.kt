package org.firstinspires.ftc.teamcode.Auton.MainAutons

//import org.firstinspires.ftc.teamcode.subsystems.lower.LowerMotorSubsystem

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Auton.AutonPositions
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.cv.CvBallDetectionP
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.BORD_Y
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distAndVeloToNewThetaClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distAndVeloToNewThetaFar
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToTimeClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToTimeFar
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToVelocityClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToVelocityFar
import org.firstinspires.ftc.teamcode.utils.Lefile
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import java.io.File
import kotlin.math.atan2
import kotlin.math.hypot

//Auton Naming Convention
//total slots = 4: __ __ __ __
//First slot = Name Type: Auton
//2nd slot = Side type: Blue, Red
//3rd slot = Classifier type (There can be multiple types on the same auto):
//1. Leaving it blank
//2. Wait (only for shooter type autons)
//3. Far (only for shooter and backup type autons)
//4. Close (only for shooter and backup type autons)
//4th slot = Auton type: Motif, Backup, Shooter, Artifact, CoOp
//Example Auton = AutonBlueCloseBackup, AutonRedWaitFarShooter ...
//Main Autons should be: Auton__ __Artifact & Auton__ __ CoOp
@Autonomous(name = "[COOP-21-B] Auton Blue Far CoOp", group = "Auton")
@Configurable
class AutonBlueFarCoOp: NextFTCOpMode(){ //Pretend robot is 14 to 16 (14 is intake to backplate)
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
    //    var follower : Follower? = null
    /////////////////
    ////Constants////
    /////////////////
    companion object {
        val delayStartShoot: Double = 0.6
        val DelayBeforeShoot = 0.0
        val delayAfterEachShoot = 0.27;
        val DelayInIntake = 0.64;

        var canShoot: Boolean = false
        val goalX = 2.5
        val goalY = 144.0 - 6.0
        var pathStarted = false;

        var distanceToBlob: Double = Double.MAX_VALUE
        var radiansToRotateToBlob: Double = Double.MAX_VALUE
//        var directionGoalX = 4.0;
//        var directionGoalY = 144.0-4.0;
    }


    /////////////////
    ////Positions////
    /////////////////
    //Constant positions

    /////////////
    ////Paths////
    /////////////
    // The different paths the robot will take in during Auton
//    private var pushPath: PathChain? = null
//
//    private var pushToClose: PathChain? = null;
//
//    private var closeToL2: PathChain? = null
//    private var L2Shoot: PathChain? = null
//
//    private var closeToGate: PathChain? = null
//    private var gateToAfter: PathChain? = null
//
//    private var gateToShoot: PathChain? = null
//
//    private var closeToL1: PathChain? = null
//    private var L1Shoot: PathChain? = null
//
//    private var closeToL3: PathChain? = null
//    private var L3Shoot: PathChain? = null
//
//    private var closeToHPZ: PathChain? = null
//    private var HPZShoot: PathChain? = null
//
//    private var parkPath: PathChain? = null

//    private val LOW_BRAKING = 0.6;
//    private val HIGH_BRAKING = 0.8;

    private val LOW_BRAKING = 0.75;
    private val HIGH_BRAKING = 0.75;

    private var farToL3: PathChain? = null
    private var farToHPZ: PathChain? = null;
    private var L3toFar: PathChain? = null;
    private var HPZtoFar: PathChain? = null;
    private var parkPath: PathChain? = null;

    ////////////////////
    ////Path Builder////
    ////////////////////
    fun buildPaths() {
        farToL3 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.startPoseFarPush),
                    AutonPositions.Blue(AutonPositions.L3IntakeControlPose),
                    AutonPositions.Blue(AutonPositions.L3IntakePose)
                )
            )
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(Math.toRadians(270.0)),
                AutonPositions.Blue(Math.toRadians(180.0)),
                0.5
            )
            .addParametricCallback(0.59, IntakeCommand)
            .addParametricCallback(0.64, IntakeAfterCommand)
            .build()
        L3toFar = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.L3IntakePose),
                    AutonPositions.Blue(AutonPositions.farShootPoseCoOp)
                )
            )
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(Math.toRadians(180.0)),
                AutonPositions.Blue(Math.toRadians(215.0))
            )
            .build()
        farToHPZ = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.farShootPoseCoOp),
                    AutonPositions.Blue(AutonPositions.commonIntakePos)
                )
            )
            .addParametricCallback(0.64, IntakeAfterCommand)
            .setBrakingStrength(HIGH_BRAKING)
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(Math.toRadians(215.0)),
                AutonPositions.Blue(Math.toRadians(180.0))
            )
            .build()
        HPZtoFar = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.commonIntakePos),
                    AutonPositions.Blue(AutonPositions.farShootPoseCoOp)
                )
            )
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(Math.toRadians(180.0)),
                AutonPositions.Blue(Math.toRadians(215.0))
            )
            .build()
        parkPath = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.farShootPoseCoOp),
                    AutonPositions.Blue(AutonPositions.farParkCoOp)
                )
            )
            .setConstantHeadingInterpolation(
                AutonPositions.Blue(Math.toRadians(215.0))
            )
            .build()
    }

    val intakePower: Command = InstantCommand {PedroComponent.follower.setMaxPower(0.6)}
    val maxPower: Command = InstantCommand {PedroComponent.follower.setMaxPower(1.0)}

    val SetCanShootFalse: Command = InstantCommand {canShoot = false}
    val SetCanShootTrue: Command = InstantCommand {canShoot = true}

    val stopFollower: Command = InstantCommand {PedroComponent.follower.breakFollowing()}

    val IntakeCommand: Command
        get() = ParallelGroup(
//            SetCanShootTrue,
            intakePower,
            IntakeMotorSubsystem.intake,
            MagMotorSubsystem.intake,
//            MagServoSubsystem.run,
            MagblockServoSubsystem.block

        )
    val IntakeAfterCommand: Command
        get() = ParallelGroup(
//            SetCanShootTrue,
            maxPower,
            IntakeMotorSubsystem.intake,
            MagMotorSubsystem.intake,
//            MagServoSubsystem.run,
            MagblockServoSubsystem.block

        )
    val TravelCommand: Command
        get() = ParallelGroup(
            maxPower,
            IntakeMotorSubsystem.off,
            MagMotorSubsystem.off,
//            MagServoSubsystem.stop,
            MagblockServoSubsystem.block,

            )
    val ShootCommand: Command
        get() = ParallelGroup(
//            SetCanShootFalse,
            maxPower,
            MagblockServoSubsystem.unblock,
            MagMotorSubsystem.On(1.0),
            IntakeMotorSubsystem.intake,
//            MagServoSubsystem.run
        )

    fun robotShoot(): Command {
        return SequentialGroup(
            Delay(DelayBeforeShoot),
            ShootCommand,
            Delay(delayAfterEachShoot),
            stopFollower,
            TravelCommand,
        )
    }

    fun robotIntake(followedPath: PathChain?): Command {
        return SequentialGroup(
            TravelCommand,
            FollowPath(followedPath!!)
        )
    }

    fun getBlobPath(): PathChain {
        AutonPositions.Blue(AutonPositions.commonIntakePos)
        val robotPos: Pose = AutonPositions.Blue(AutonPositions.farShootPoseCoOp)
        var intakePos: Pose

        if (distanceToBlob != -1.0) {
            val xa: Double = Math.sin(radiansToRotateToBlob) * distanceToBlob
            val ya: Double = Math.cos(radiansToRotateToBlob) * distanceToBlob
            intakePos = Pose(robotPos.x + xa, robotPos.y + ya)

            return PedroComponent.follower.pathBuilder()
                .addPath(
                    BezierLine(
                        robotPos,
                        intakePos
                    )
                )
                .setTangentHeadingInterpolation()
                .build()
        }

        return farToHPZ!!
    }

    fun robotGoShoot(followedPath: PathChain?): Command {
        return ParallelGroup(
            SequentialGroup(
                IntakeAfterCommand,
                Delay(DelayInIntake),
                TravelCommand,
            ),
            FollowPath(followedPath!!, true)
        )
    }


    /////////////////////////
    ////Autonomous Runner////
    /////////////////////////
    var stopShooterAutoAim = false;
    private val autonomousRoutine: Command
        get() = SequentialGroup(
            TravelCommand,
            Delay(delayStartShoot),
            robotShoot(),

            robotIntake(farToHPZ),
            robotGoShoot(HPZtoFar),
            robotShoot(),

            /////////////////////////
            ////REPEATING SECTION////
            /////////////////////////
            robotIntake(getBlobPath()),
            robotGoShoot(HPZtoFar),
            robotShoot(),
            /////////////////////////
            ////REPEATING SECTION////
            /////////////////////////
            robotIntake(getBlobPath()),
            robotGoShoot(HPZtoFar),
            robotShoot(),
            /////////////////////////
            ////REPEATING SECTION////
            /////////////////////////
            robotIntake(getBlobPath()),
            robotGoShoot(HPZtoFar),
            robotShoot(),
            /////////////////////////
            ////REPEATING SECTION////
            /////////////////////////
            robotIntake(getBlobPath()),
            robotGoShoot(HPZtoFar),
            robotShoot(),
            /////////////////////////
            ////REPEATING SECTION////
            /////////////////////////

            robotIntake(farToL3),
            robotGoShoot(L3toFar),
            robotShoot(),

            // Park
            TravelCommand,
            InstantCommand { stopShooterAutoAim = true },
            ShooterSubsystem.On(9999.0),
            FollowPath(parkPath!!),
        )

    // for auto aim in autons, copy this...
    var vxOld = listOf(0.0, 0.0, 0.0);
    var vyOld = listOf(0.0, 0.0, 0.0);
    var lastPose: Pose = Pose(0.0, 0.0, 0.0);
    var lastTime = 0.0;
    override fun onUpdate() {
        val dx = goalX - PedroComponent.follower.pose.x
        val dy = goalY - PedroComponent.follower.pose.y
        val dxy = hypot(dx, dy)
        if (vxOld.isEmpty() || vyOld.isEmpty()) {
            vxOld = listOf(0.0, 0.0, 0.0);
            vyOld = listOf(0.0, 0.0, 0.0);
        } else {
            vxOld = vxOld.slice(IntRange(1, vxOld.lastIndex)) + listOf((PedroComponent.follower.pose.x - lastPose.pose.x) / (runtime - lastTime));
            vyOld = vyOld.slice(IntRange(1, vyOld.lastIndex)) + listOf((PedroComponent.follower.pose.y - lastPose.pose.y) / (runtime - lastTime));
        }
        val vx = vxOld.average();
        val vy = vyOld.average();
        val dxp = dx - 1.0 * vx * (if (PedroComponent.follower.pose.y < BORD_Y) distanceToTimeFar(dxy) else distanceToTimeClose(dxy))
        val dyp = dy - 1.0 * vy * (if (PedroComponent.follower.pose.y < BORD_Y) distanceToTimeFar(dxy) else distanceToTimeClose(dxy))
        val dxyp = hypot(dxp, dyp)
        ShooterSubsystem.AutoAim(
            dxyp,
            { dist ->
                (
                        if (PedroComponent.follower.pose.y < BORD_Y)
                            distanceToVelocityFar(dist)
                        else
                            distanceToVelocityClose(dist)
                        )
            }
        )()
        TurretThetaSubsystem.SetThetaPos(
            (
                    if (PedroComponent.follower.pose.y < BORD_Y)
                        distAndVeloToNewThetaFar(dxyp, ShooterSubsystem.velocity)
                    else
                        distAndVeloToNewThetaClose(dxyp, ShooterSubsystem.velocity)
                    )
        )()
        TurretPhiSubsystem.AutoAim(
            dxp,
            dyp,
            PedroComponent.follower.heading.rad
        )()
        lastPose = PedroComponent.follower.pose;
        lastTime = runtime;

        // ...to this

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
//        follower = Constants.createFollower(hardwareMap)

        PedroComponent.follower.setStartingPose(
            AutonPositions.Blue(AutonPositions.startPoseFarPush)
        )

        TurretPhiSubsystem.zero();
    }

    /** This method is called continuously after Init while waiting for "play".  */
    override fun onWaitForStart() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system  */
    override fun onStartButtonPressed() {
        autonomousRoutine()

        opmodeTimer!!.resetTimer()
        actionTimer!!.resetTimer()

        pathStarted = true;

        //////////////////////////////////////////////////////////
        /////////////// ARJANS WEIRD ASS BLOB CODE ///////////////
        //////////////////////////////////////////////////////////
        var Portal: CvBallDetectionP? = null
        var Blobs: MutableList<ColorBlobLocatorProcessor.Blob>? = null
        var cd = 0.0

        Portal = CvBallDetectionP( hardwareMap)
        waitForStart()


        while (opModeIsActive()) {
            var min = Double.Companion.MAX_VALUE // reset each loop iteration
            var Minb: ColorBlobLocatorProcessor.Blob? = null

            Portal.updateDetections()
            Blobs = Portal.getBlobs()

            if (Blobs != null && !Blobs.isEmpty()) {
                telemetry.addData("Blob count", Blobs.size)
                if (Blobs.isEmpty()) {
                    cd = -1.0
                }
                for (b in Blobs) {
                    val circleFit = b.getCircle()

                    if (circleFit == null) continue

                    val radius = circleFit.getRadius().toDouble()

                    if (radius == 0.0) continue
                    cd = ((120.0 * 529) / (radius * 2)) * 2
                    val theta = Math.toDegrees(atan2((circleFit.getX() - 320).toDouble(), 391.0))

                    telemetry.addData(
                        "Blob @ X=" + circleFit.getX().toInt() + " Circularity",
                        b.getCircularity()
                    )
                    telemetry.addData("Blob @ X=" + circleFit.getX().toInt() + " Radius", radius)
                    telemetry.addData(
                        "Blob @ X=" + circleFit.getX().toInt() + " Y",
                        circleFit.getY()
                    )
                    telemetry.addData("Blob @ X=" + circleFit.getX().toInt() + " Distance", cd)
                    telemetry.addData("Blob @ X=" + circleFit.getX().toInt() + " Angle", theta)

                    if (cd < min) {
                        min = cd
                        distanceToBlob = cd
                        radiansToRotateToBlob = theta
                        Minb = b
                    }
                }
            }

            telemetry.update()
        }
        //////////////////////////////////////////////////////////
        /////////////// ARJANS WEIRD ASS BLOB CODE ///////////////
        //////////////////////////////////////////////////////////
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