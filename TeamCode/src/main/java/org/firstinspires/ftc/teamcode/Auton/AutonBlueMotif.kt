package org.firstinspires.ftc.teamcode.Auton

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.PedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem.open
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem.close
import org.firstinspires.ftc.teamcode.subsystems.outtake.TurretSubsystem

class AutonBlueMotif : NextFTCOpMode() {
    private var pathTimer: Timer? = null
    val actionTimer: Timer? = null;
    var opmodeTimer: Timer? = null;

    init {
        addComponents(
            SubsystemComponent(LowerSubsystem, OuttakeSubsystem),
            PedroComponent(Constants::createFollower)
        );

        pathTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer!!.resetTimer()
    }

    enum class AutonPath {
        StartingPath, //the starting path of the robot

        GPPPathIntake1, //GPP Intakes 1st row of artifacts and then goes shoot
        GPPPathIntake2, //GPP Intakes 2nd row of artifacts
        GPPPathLastShoot, //GPP last thing to shoot before going to go park

        PGPPathStart, //PGP start path
        PGPPathIntake1, //PGP Intakes 1st row of artifacts and then goes shoot and prepares to intake 2
        PGPPathIntake2, //PGP Intakes 2nd row of artifacts and then goes shoot before parking

        PPGPathIntakePrep1, //PPG what it does before it starts to intake
        PPGPathIntake1, //PPG Intakes 1st row of artifacts and then goes shoot
        PPGPathIntake2, //PPG Intakes 2nd row of artifacts
        PPGPathLastShoot, //PPG last thing to shoot before going to go park

        ParkPath, //the last path of the robot, goes park
        EndAuton
    }
    private var pathState: AutonPath = AutonPath.StartingPath

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     */
    fun setPathState(pState: AutonPath) {
        pathState = pState
        pathTimer?.resetTimer()
    }

    val follower = PedroComponent.follower;

    /////////////////
    ////Positions////
    /////////////////
    // Positions the robot will be in during Auton
    // robotStartPath positions
    private val startPose = Pose(33.0, 136.0, Math.toRadians(0.0)) // Start Pose of our robot.
    private val startPathControlP = Pose(58.0, 108.0)
    private val endBeforeMotifStartPose = Pose(25.0, 100.0, Math.toRadians(270.0)) // End Pose of our robot.

    /**MOTIF GPP/// */ // MotifGPPIntake1
    private val motifGPPIntake1IntakedControlP = Pose(73.0, 87.0)
    private val motifGPPIntake1Intaked = Pose(20.0, 81.0, Math.toRadians(180.0))

    private val motifGPPIntake1ToShoot = Pose(58.0, 94.0)

    //MotifGPPIntake2 (also where it shoots the 2nd time)
    private val motifGPPIntake2Purp1ControlP = Pose(95.0, 134.0)
    private val motifGPPIntake2Purp1Intaked = Pose(37.0, 56.0)

    private val motifGPPIntake2RestOTPControlP = Pose(60.0, 90.0)
    private val motifGPPIntake2RestOfThemPrep = Pose(18.536, 69.658, Math.toRadians(-70.0))

    private val motifGPPIntake2Intaked = Pose(19.364, 68.7)
    private val motifGPPIntake2Intaked2 = Pose(29.0, 67.9)

    /**MOTIF PGP/// */ // MotifPGPIntake1
    private val motifPGPIntake1PrepControlP = Pose(43.0, 98.0)
    private val motifPGPIntake1Prep = Pose(37.0, 84.0, Math.toRadians(180.0))

    private val motifPGPIntake1IntakedControl = Pose(32.0, 112.0)
    private val motifPGPIntake1Intaked = Pose(17.45, 93.0, Math.toRadians(-70.0))

    private val motifPGPIntake1Intaked2 = Pose(20.9, 90.9)

    private val motifPGPIntake1FinalPos = Pose(56.0, 94.0)

    // MotifPGPIntake2
    private val motifPGPIntake2PrepControlP = Pose(101.0, 120.0)
    private val motifPGPIntake2Prep = Pose(37.0, 60.0, Math.toRadians(180.0))

    private val motifPGPIntake2IntakedControlP = Pose(1.5, 58.0)


    /**MOTIF PPG/// */ // MotifPPGIntake1 prep
    private val motifPPGIntake1StartPos = Pose(18.2, 92.6, Math.toRadians(-65.0))
    private val motifPPGIntake1MidPos = Pose(18.2, 89.5)

    // MotifPPGIntake 1 position
    private val motifPPGIntake1EndPos = Pose(59.0, 94.0, Math.toRadians(0.0))

    // MotifPPGIntake2 intake preparation
    private val motifPPGIntake2ControlP1 = Pose(94.0, 93.0)
    private val motifPPGIntake2MidPos = Pose(37.0, 68.0, Math.toRadians(180.0))

    // MotifPPGIntake2 positions
    private val motifPPGIntake2PushOutPurple = Pose(37.0, 57.5)
    private val motifPPGIntake2EndControlP1 = Pose(45.0, 88.0)
    private val motifPPGIntake2End = Pose(37.0, 42.0, Math.toRadians(270.0))


    /**Motif Park/// */
    private val motifsShooterEnd = Pose(75.0, 90.0, Math.toRadians(180.0))

    private val motifBlueParkControlP = Pose(54.0, 14.0)
    private val motifBluePark = Pose(16.0, 5.0)


    /////////////
    ////Paths////
    /////////////
    // The different paths the robot will take in during Auton
    private var robotStartPath: Path? = null

    //Motif GPP Paths
    private var motifGPPIntake1: PathChain? = null
    private var motifGPPIntake2: PathChain? = null
    private var motifGPPLastShot: Path? = null

    //Motif PGP Paths
    private var motifPGPIntake1: PathChain? = null
    private var motifPGPIntake2: PathChain? = null

    //Motif PPG Paths
    private var motifPPGIntake1Prep: PathChain? = null
    private var motifPPGIntake1: Path? = null
    private var motifPPGIntake2: PathChain? = null
    private var motifPPGLastShot: Path? = null

    private var motifPark: Path? =
        null //wants to put end pos in the folder: org.firstinspires.ftc.teamcode


    ////////////////////
    ////Path Builder////
    ////////////////////
    // Builds the aforementioned paths with the initialized positions
    fun buildPaths() {
        /* This is the path before all the motif stuff. */
        robotStartPath = Path(
            BezierCurve(
                startPose,
                startPathControlP,
                endBeforeMotifStartPose
            )
        )
        robotStartPath!!.setLinearHeadingInterpolation(
            startPose.heading,
            endBeforeMotifStartPose.heading
        )

        // 1st Pattern motif, Green-Purple-Purple
        motifGPPIntake1 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    endBeforeMotifStartPose,
                    motifGPPIntake1IntakedControlP,
                    motifGPPIntake1Intaked
                )
            )
            .setLinearHeadingInterpolation(
                endBeforeMotifStartPose.heading,
                motifGPPIntake1Intaked.heading
            )

            .addPath(
                BezierLine(
                    motifGPPIntake1Intaked,
                    motifGPPIntake1ToShoot
                )
            )
            .build()

        motifGPPIntake2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    motifGPPIntake1ToShoot,
                    motifGPPIntake2Purp1ControlP,
                    motifGPPIntake2Purp1Intaked
                )
            ) //                .setLinearHeadingInterpolation(endBeforeMotifStartPose.heading, motifGPPIntake1Intaked.heading)

            .addPath(
                BezierCurve(
                    motifGPPIntake2Purp1Intaked,
                    motifGPPIntake2RestOTPControlP,
                    motifGPPIntake2RestOfThemPrep
                )
            )
            .setLinearHeadingInterpolation(
                motifGPPIntake1Intaked.heading,
                motifGPPIntake2RestOfThemPrep.heading
            )

            .addPath(
                BezierLine(
                    motifGPPIntake2RestOfThemPrep,
                    motifGPPIntake2Intaked
                )
            )
            .addPath(
                BezierLine(
                    motifGPPIntake2Intaked,
                    motifGPPIntake2Intaked2
                )
            )
            .build()

        motifGPPLastShot = Path(
            BezierLine(
                motifGPPIntake2Intaked,
                motifsShooterEnd
            )
        )
        motifPPGLastShot!!.setLinearHeadingInterpolation(
            motifGPPIntake2RestOfThemPrep.heading,
            motifsShooterEnd.heading
        )

        // 2nd Pattern motif, Purple-Green-Purple
        motifPGPIntake1 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    endBeforeMotifStartPose,
                    motifPGPIntake1PrepControlP,
                    motifPGPIntake1Prep
                )
            )
            .setLinearHeadingInterpolation(
                endBeforeMotifStartPose.heading,
                motifPGPIntake1Prep.heading
            )

            .addPath(
                BezierCurve(
                    motifPGPIntake1Prep,
                    motifPGPIntake1IntakedControl,
                    motifPGPIntake1Intaked
                )
            )
            .setLinearHeadingInterpolation(
                motifPGPIntake1Prep.heading,
                motifPGPIntake1Intaked.heading
            )

            .addPath(
                BezierLine(
                    motifPGPIntake1Intaked,
                    motifPGPIntake1Intaked2
                )
            )

            .addPath(
                BezierLine(
                    motifPGPIntake1Intaked2,
                    motifPGPIntake1FinalPos
                )
            )
            .build()

        motifPGPIntake2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    motifPGPIntake1FinalPos,
                    motifPGPIntake2PrepControlP,
                    motifPGPIntake2Prep
                )
            )
            .setLinearHeadingInterpolation(
                motifPGPIntake1Intaked.heading,
                motifPGPIntake2Prep.heading
            )

            .addPath(
                BezierCurve(
                    motifPGPIntake2Prep,
                    motifPGPIntake2IntakedControlP,
                    motifsShooterEnd
                )
            )
            .build()

        // 3rd Pattern motif, Purple-Purple-Green for now all paths are combined, but in future this will probably be separated.
        motifPPGIntake1Prep = follower.pathBuilder()
            .addPath(
                BezierLine(
                    endBeforeMotifStartPose,
                    motifPPGIntake1StartPos
                )
            )
            .setLinearHeadingInterpolation(
                endBeforeMotifStartPose.heading,
                motifPPGIntake1StartPos.heading
            )

            .addPath(
                BezierLine(
                    motifPPGIntake1StartPos,
                    motifPPGIntake1MidPos
                )
            )
            .build()

        motifPPGIntake1 = Path(
            BezierLine(
                motifPPGIntake1MidPos,
                motifPPGIntake1EndPos
            )
        )
        robotStartPath!!.setLinearHeadingInterpolation(
            motifPPGIntake1StartPos.heading,
            motifPPGIntake1EndPos.heading
        )

        motifPPGIntake2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    motifPPGIntake1EndPos,
                    motifPPGIntake2ControlP1,
                    motifPPGIntake2MidPos
                )
            )
            .setLinearHeadingInterpolation(
                motifPPGIntake1EndPos.heading,
                motifPPGIntake2MidPos.heading
            )

            .addPath(
                BezierLine(
                    motifPPGIntake2MidPos,
                    motifPPGIntake2PushOutPurple
                )
            )

            .addPath(
                BezierCurve(
                    motifPPGIntake2PushOutPurple,
                    motifPPGIntake2EndControlP1,
                    motifPPGIntake2End
                )
            )
            .setLinearHeadingInterpolation(
                motifPPGIntake2MidPos.heading,
                motifPPGIntake2End.heading
            )
            .build()

        motifPPGLastShot = Path(
            BezierLine(
                motifPPGIntake2End,
                motifsShooterEnd
            )
        )
        motifPPGLastShot!!.setLinearHeadingInterpolation(
            motifPPGIntake2End.heading,
            motifsShooterEnd.heading
        )


        //last path to follow, the park path
        motifPark = Path(
            BezierCurve(
                motifsShooterEnd,
                motifBlueParkControlP,
                motifBluePark
            )
        )
    }


    /////////////////////
    ////State Manager////
    /////////////////////
    // Controls which path the robot will take after finishing a specific path.
    fun autonomousPathUpdate() {
        when (pathState) {
            AutonPath.StartingPath -> {
                open.schedule()
                FollowPath(robotStartPath!!)
                setPathState(AutonPath.GPPPathIntake1) //for now 3
            }
            //GPP PATHS (START)
            AutonPath.GPPPathIntake1 -> if (!follower.isBusy) {
                close.schedule()
//                IntakeSubsystem.forward.schedule() //(TURNS ON INTAKE) temporary value
                FollowPath(motifGPPIntake1!!)
                if (follower.atPose(endBeforeMotifStartPose, 1.0, 1.0)) {
                    setPathState(AutonPath.GPPPathIntake2)
                }
            }

            AutonPath.GPPPathIntake2 -> if (!follower.isBusy) {
                FollowPath(motifGPPIntake2!!)
                if (follower.atPose(motifGPPIntake1ToShoot, 1.0, 1.0)) {
                    IntakeSubsystem.reverse.schedule() //(TURNS OFF INTAKE) temporary value
                    open.schedule()
                }
                if (follower.atPose(motifGPPIntake2Purp1Intaked, 5.0, 5.0)) {
                    close.schedule()
//                    IntakeSubsystem.forward.schedule() //(TURNS ON INTAKE) temporary value
                }
                if (follower.atPose(motifGPPIntake2Intaked2, 1.0, 1.0)) {
                    setPathState(AutonPath.GPPPathLastShoot)
                }
            }

            AutonPath.GPPPathLastShoot -> if (!follower.isBusy) {
                FollowPath(motifGPPLastShot!!)
                setPathState(AutonPath.ParkPath)
            }
            //GPP PATHS (END)

            //PGP PATHS (START)
            AutonPath.PGPPathStart -> if (!follower.isBusy) {
                close.schedule()
//                IntakeSubsystem.forward.schedule() //(TURNS ON INTAKE) temporary value
//                FollowPath(motifPGPIntake1!!)
                setPathState(AutonPath.PGPPathIntake1)
//                if (follower.atPose(endBeforeMotifStartPose, 1.0, 1.0)) {
//
//                }
            }

            AutonPath.PGPPathIntake1 -> if (!follower.isBusy) {
                FollowPath(motifPGPIntake1!!)
                if (follower.atPose(motifPGPIntake1FinalPos, 1.0, 1.0)) {
                    IntakeSubsystem.reverse.schedule() //(TURNS OFF INTAKE) temporary value
                }
                setPathState(AutonPath.PGPPathIntake2)
            }

            AutonPath.PGPPathIntake2 -> if (!follower.isBusy) {
                FollowPath(motifPGPIntake2!!)
                open.schedule()
                if (follower.atPose(motifPGPIntake2Prep, 5.0, 5.0)) {
                    close.schedule()
//                    IntakeSubsystem.forward.schedule() //(TURNS ON INTAKE) temporary value
                }
                if (follower.atPose(motifsShooterEnd, 1.0, 1.0)) {
                    IntakeSubsystem.reverse.schedule() //(TURNS OFF INTAKE) temporary value
                }
                if (follower.atPose(motifsShooterEnd, 1.0, 1.0)) {
                    setPathState(AutonPath.ParkPath)
                }
            }
            //PGP PATHS (END)

            //PPG PATHS (START)
            AutonPath.PPGPathIntakePrep1 -> if (!follower.isBusy) {
                close.schedule()
//                IntakeSubsystem.forward.schedule() //(TURNS ON INTAKE) temporary value
                FollowPath(motifPPGIntake1Prep!!)
                if (follower.atPose(endBeforeMotifStartPose, 1.0, 1.0)) {
                    setPathState(AutonPath.PPGPathIntake1)
                }
            }

            AutonPath.PPGPathIntake1 -> if (!follower.isBusy) {
                FollowPath(motifPPGIntake1!!)
                if (follower.atPose(motifPPGIntake1EndPos, 1.0, 1.0)) {
                    IntakeSubsystem.reverse.schedule() //(TURNS OFF INTAKE) temporary value
                }
                if (follower.atPose(motifPPGIntake1EndPos, 1.0, 1.0)) {
                    setPathState(AutonPath.PPGPathIntake2)
                }
            }

            AutonPath.PPGPathIntake2 -> if (!follower.isBusy) {
                open.schedule()
                if (follower.atPose(motifPPGIntake2MidPos, 1.0, 1.0)) {
                    close.schedule()
//                    IntakeSubsystem.forward.schedule() //(TURNS ON INTAKE) temporary value
                }
                FollowPath(motifPPGIntake2!!)
                if (follower.atPose(motifPPGIntake2End, 1.0, 1.0)) {
                    IntakeSubsystem.reverse.schedule() //(TURNS OFF INTAKE) temporary value
                }
                if (follower.atPose(motifPPGIntake2End, 1.0, 1.0)) {
                    setPathState(AutonPath.PPGPathLastShoot)
                }
            }

            AutonPath.PPGPathLastShoot -> if (!follower.isBusy) {
                FollowPath(motifPPGLastShot!!)
                setPathState(AutonPath.ParkPath)
            }
            //PPG PATHS (END)

            AutonPath.ParkPath -> if (!follower.isBusy) {
                open.schedule()
                if (pathTimer!!.elapsedTimeSeconds > 2.5) {
                    FollowPath(motifPark!!)
                }
                setPathState(AutonPath.EndAuton) //last path
            }

            AutonPath.EndAuton -> {
                //do jack
            }
        }
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play".  */
    override fun runOpMode() {
        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
        autonomousPathUpdate()
        //        if (pathTimer.getElapsedTimeSeconds() > 1) {
//            CommandManager.INSTANCE.scheduleCommand(TurretSubsystem.INSTANCE.autoAim(telemetry));
//        }
//        TurretSubsystem.AutoAim(
//            { 0.0 },
//            { 0.0 },
//            { 0.0.rad }
//        ).schedule() // todo: wait for localization
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState)
        telemetry.addData("x", follower.pose.x)
        telemetry.addData("y", follower.pose.y)
        telemetry.addData("heading", follower.pose.heading)
        telemetry.update()
    }

    /** This method is called continuously after Init while waiting for "play". **/
    override fun onInit() {
        //        follower = Constants.createFollower(hardwareMap);
        buildPaths()
        follower.setStartingPose(startPose)
    }

    /** This method is called continuously after Init while waiting for "play".  */
    override fun onWaitForStart() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system  */
    override fun onStartButtonPressed() {
        opmodeTimer!!.resetTimer()
        setPathState(AutonPath.StartingPath)
    }

    /** We do not use this because everything should automatically disable  */
    override fun onStop() {}  // todo: log to file
}