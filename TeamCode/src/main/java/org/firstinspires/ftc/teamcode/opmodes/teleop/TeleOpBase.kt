package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.FieldCentric
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.Auton.AutonPositions
import org.firstinspires.ftc.teamcode.Auton.AutonPositions.Pos
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleOp.Companion.shootAngleVal
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleOp.Companion.shootDelay
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.BORD_Y
import org.firstinspires.ftc.teamcode.utils.Lefile
import java.io.File
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt
import kotlin.math.sin


data class ResetModeParams(val x: Double, val y: Double, val h: Angle)

const val D_OFS = 0.0;

@Configurable
open class TeleOpBase(
    private val isBlue: Boolean,
    private val goalX: Double,
    private val goalY: Double,
    private val resetModeParams: ResetModeParams,
    private val resetModePhiAngle: Angle,
    private val distanceToVelocityClose: (Double) -> Double,
    private val distAndVeloToThetaClose: (Double, Double) -> Double,
    private val distanceToVelocityFar: (Double) -> Double,
    private val distAndVeloToThetaFar: (Double, Double) -> Double,
    private val distanceToTimeClose: (Double) -> Double,
    private val distanceToTimeFar: (Double) -> Double
): NextFTCOpMode() {

//    private val lfw = MotorEx("lfw").reversed();
//    private val lbw = MotorEx("lbw").reversed();
//    private val rfw = MotorEx("rfw");
//    private val rbw = MotorEx("rbw");

//    private var odom: OdometrySubsystem? = null;
    val x:  Double get() { return (PedroComponent.follower.pose.x + ofsX);}
    val y:  Double get() { return (PedroComponent.follower.pose.y + ofsY);}
    val h:  Angle  get() { return (PedroComponent.follower.pose.heading).rad + ofsH; }
    var vx = 0.0;
    var vy = 0.0;
    var vxOld = listOf(0.0, 0.0, 0.0);
    var vyOld = listOf(0.0, 0.0, 0.0);
    var ax = 0.0;
    var ay = 0.0
    var axOld = listOf(0.0, 0.0, 0.0);
    var ayOld = listOf(0.0, 0.0, 0.0);

//    private var ofsX = 0.0;
//    private var ofsY = 0.0;
//    private var ofsH = 0.0;

//    val x: Double
//        get() {
//            if (odom != null) {
//                return odom!!.rOx1 + ofsX;
//            }
//            return 0.0;
//        }
//    val y: Double
//        get() {
//            if (odom != null) {
//                return odom!!.rOy1 + ofsY;
//            }
//            return 0.0;
//        }
//    val h: Angle
//        get() {
//            if (odom != null) {
//                return odom!!.rOh.rad + ofsH.rad;
//            }
//            return 0.0.rad;
//        }
//    val vx: Double
//        get() {
//            if (odom != null) {
//                return odom!!.vx;
//            }
//            return 0.0;
//        }
//    val vy: Double
//        get() {
//            if (odom != null) {
//                return odom!!.vy;
//            }
//            return 0.0;
//        }
//    val vh: Angle
//        get() {
//            if (odom != null) {
//                return odom!!.omega.rad;
//            }
//            return 0.0.rad;
//        }
//    val ax: Double
//        get() {
//            if (odom != null) {
//                return odom!!.ax;
//            }
//            return 0.0;
//        }
//    val ay: Double
//        get() {
//            if (odom != null) {
//                return odom!!.ay;
//            }
//            return 0.0;
//        }

    var gatePreIntakeChain: PathChain? = null;
    var gateIntakeChain: PathChain? = null;
    var farShootChain: PathChain? = null;
    var closeShootChain: PathChain? = null;
    //    var closeIntakeChain: PathChain;
    var driverControlled: PedroDriverControlled? = null;
    //    var driverControlled: PedroDriverControlled;
    var parkChain: PathChain? = null;

//    var cv: CVSubsystem_VisionPortal? = null;

    init {
        addComponents(
            SubsystemComponent(
                LowerSubsystem,
                OuttakeSubsystem
            ),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )
    }

    var lastPose: Pose = Pose(0.0, 0.0, 0.0);
    var lastVX = 0.0;
    var lastVY = 0.0;
    var lockDirection = false;
    var lastTime = 0.0;
    override fun onInit() {
        ShooterSubsystem.off()
        MagMotorSubsystem.off()
        TurretPhiSubsystem.started = false;
        PedroComponent.follower.update()
//        MagServoSubsystem.stop()

        // ROBOT CENTRIC:
//        val scaleDrive: (Double) -> Double = { inp -> (inp - 0.1) / 0.9 + 0.1 } // [0.0, 0.1] deadzone (0.0 power), [0.1, 0.9] 0.0 --> 1.0 power, [0.9, 1.0] 1.0 power
//        driverControlled = PedroDriverControlled(
//            -Gamepads.gamepad1.leftStickY.deadZone(0.1).map { scaleDrive(it) * speedFactorDrive },
//            -Gamepads.gamepad1.leftStickX.deadZone(0.1).map { scaleDrive(it) * speedFactorDrive },
//            -Gamepads.gamepad1.rightStickX.deadZone(0.1).map { scaleDrive(it) * speedFactorDrive },
//            true
//        )
        // FIELD CENTRIC:
        driverControlled = PedroDriverControlled(
            Gamepads.gamepad1.leftStickY.deadZone(0.02).map { (if (abs(gamepad1.left_stick_x) > 0.9) 0.0 else (if (isBlue) it else -it)) * speedFactorDrive },
            Gamepads.gamepad1.leftStickX.deadZone(0.02).map { (if (abs(gamepad1.left_stick_y) > 0.9) 0.0 else (if (isBlue) it else -it)) * speedFactorDrive },
            -Gamepads.gamepad1.rightStickX.deadZone(0.02).map { it * speedFactorDrive },
            false
        )
//        driverControlled = PedroDriverControlled(
//            {
//                val rawX = gamepad1.left_stick_x.toDouble()
//                val rawY = gamepad1.left_stick_y.toDouble()
//                val mag = hypot(rawX, rawY)
//
//                var finalY = gamepad1.left_stick_y.toDouble()
//
//                if (mag > 0.9) {
//                    val snapped = snapVectorTo45(rawX, rawY)
//                    finalY = snapped.second // Snapped Y
//                }
//
//                (if (isBlue) finalY else -finalY) * speedFactorDrive
//            },
//            {
//                val rawX = gamepad1.left_stick_x.toDouble()
//                val rawY = gamepad1.left_stick_y.toDouble()
//                val mag = hypot(rawX, rawY)
//
//                var finalX = gamepad1.left_stick_y.toDouble()
//
//                if (mag > 0.9) {
//                    val snapped = snapVectorTo45(rawX, rawY)
//                    finalX = snapped.first // Snapped X
//                }
//
//                (if (isBlue) finalX else -finalX) * speedFactorDrive
//            },
//            Gamepads.gamepad1.rightStickX.map {
//                if (abs(gamepad1.right_stick_x) < 0.02) {
//                    // When locking, we manually calculate heading error (this is a simple P-loop)
//                    val currentH = PedroComponent.follower.pose.heading
//                    val targetH = getSnappedHeading(currentH)
//                    var error = targetH - currentH
//
//                    // Normalize angle error to -PI to PI
//                    while (error > Math.PI) error -= 2 * Math.PI
//                    while (error < -Math.PI) error += 2 * Math.PI
//
//                    (error * 2.0).coerceIn(-1.0, 1.0) // 2.0 is your 'P' gain for snapping
//                } else {
//                    -it * speedFactorDrive
//                }
//            },
//            false
//        )
//
        gateIntakeChain = PedroComponent.follower.pathBuilder()
            .addPath(
                Path(
                    BezierLine(
                        PedroComponent.follower::getPose,
                        Pos(AutonPositions.gateOpenPoseTele, isBlue)
                    )
                )
            )
            .setConstantHeadingInterpolation(
                Pos(AutonPositions.gateOpenPoseTele, isBlue).heading
            )
            .setNoDeceleration()  // todo: temp test
            .build()
//        val gateIntakeChain = PedroComponent.follower.pathBuilder()
//            .addPath(
//                Path(
//                    BezierLine(
//                        PedroComponent.follower::getPose,
//                        Pos(AutonPositions.gateOpenPrePoseTele, isBlue)
//                    )
//                )
//            )
//            .setNoDeceleration()
//            .setHeadingInterpolation(
//                HeadingInterpolator.tangent
//            )
////            .setHeadingInterpolation(
////                HeadingInterpolator.piecewise(
////                    HeadingInterpolator.PiecewiseNode(
////                        0.0,
////                        0.67,
////                        HeadingInterpolator.tangent
////                    )
////                ),
////                HeadingInterpolator.piecewise(
////                    HeadingInterpolator.PiecewiseNode(
////                        0.67,
////                        1.0,
////
////                    )
////                )
////            )
////            .setLinearHeadingInterpolation(
////                PedroComponent.follower.heading % (2 * Math.PI),  // todo: trial
////                Pos(AutonPositions.gateOpenPoseTele, isBlue).heading,
////                0.7
////            )
////            .setHeadingInterpolation(
////                HeadingInterpolator.linearFromPoint(
////                    PedroComponent.follower::getHeading,
////                    Pos(AutonPositions.gateOpenPoseTele, isBlue).heading,
////                    0.7
////                )
////            )
////            .addPath(
////                BezierLine(
////                    Pos(AutonPositions.gateOpenPose, isBlue),
////                    Pos(AutonPositions.gateAfterOpenPose, isBlue)
////                )
////            )
////            .setLinearHeadingInterpolation(
////                Pos(AutonPositions.gateOpenPose, isBlue).heading,
////                Pos(AutonPositions.gateAfterOpenPose, isBlue).heading,
////                0.8
////            )
//            .build()
        
//
//        farShootChain = PedroComponent.follower.pathBuilder()
//            .addPath(
//                BezierLine(
//                    PedroComponent.follower::getPose,
//                    Pos(AutonPositions.shootPoseFar, isBlue)
//                )
//            )
////            .setTangentHeadingInterpolation()
//            .setHeadingInterpolation(
//                HeadingInterpolator.linearFromPoint(
//                    PedroComponent.follower::getHeading,
//                    Pos(AutonPositions.shootPoseFar, isBlue).heading,
//                    0.2
//                )
//            )
//            .build()
//
//        closeShootChain = PedroComponent.follower.pathBuilder()
//            .addPath(
//                BezierLine(
//                    PedroComponent.follower::getPose,
//                    Pos(AutonPositions.shootPoseClose, isBlue)
//                )
//            )
////            .setTangentHeadingInterpolation()
//            .setHeadingInterpolation(
//                HeadingInterpolator.linearFromPoint(
//                    PedroComponent.follower::getHeading,
//                    Pos(AutonPositions.shootPoseClose, isBlue).heading,
//                    0.2
//                )
//            )
//            .build()
//
//        parkChain = PedroComponent.follower.pathBuilder()
//            .addPath(
//                BezierLine(
//                    PedroComponent.follower::getPose,
//                    Pos(AutonPositions.parkPoseFull, isBlue)
//                )
//            )
////            .setTangentHeadingInterpolation()
//            .setHeadingInterpolation(
//                HeadingInterpolator.linearFromPoint(
//                    PedroComponent.follower::getHeading,
//                    Pos(AutonPositions.parkPoseFull, isBlue).heading,
//                    1.0
//                )
//            )
//            .build()

        val file = File(Lefile.filePath)
        while (!file.canRead()) {}
        val content = file.readText().split("\n")
        val startX = content[0].toDouble()
        val startY = content[1].toDouble()
        val startH = content[2].toDouble()

//        PedroComponent.follower.pose = Pose(72.0, 72.0, -PI / 2)
        PedroComponent.follower.setStartingPose(Pose(startX, startY, startH))
//        PedroComponent.follower.pose = Pose(startX, startY, startH)
        PedroComponent.follower.update()
//        cv = CVSubsystem_VisionPortal(startX, startY, startH, hardwareMap)

        telemetry.addData("Start X", startX);
        telemetry.addData("Start Y", startY);
        telemetry.addData("Start H (degs)", Math.toDegrees(startH));
//        odom = OdometrySubsystem(72.0, 72.0, -PI / 2, hardwareMap)
//        odom!!.updateOdom()


//        val file = File(Lefile.filePath)
//        val content = file.readText().split("\n")
//        val startX = content[0].toDouble()
//        val startY = content[1].toDouble()
//        val startH = content[2].toDouble()
//
//        odom!!.setPinpoint(startX, startY, startH)
    }

//    private fun getSnappedHeading(currentHeading: Double): Double {
//        val increment = Math.PI / 4.0 // 45 degrees
//        return (currentHeading / increment).roundToInt() * increment
//    }
//    private fun snapVectorTo45(x: Double, y: Double): Pair<Double, Double> {
//        val magnitude = hypot(x, y)
//        if (magnitude < 0.02) return Pair(0.0, 0.0)
//
//        // Get current angle of the joystick in radians
//        val angle = atan2(y, x)
//
//        // Snap angle to nearest PI/4 (45 degrees)
//        val snappedAngle = (angle / (Math.PI / 4.0)).roundToInt() * (Math.PI / 4.0)
//
//        // Reconstruct the vector using the original magnitude
//        return Pair(cos(snappedAngle) * magnitude, sin(snappedAngle) * magnitude)
//    }

    private var autoAimEnabled = true;
    private var resetMode = false;
    private var resetTypeHeading = false;

    var activeDriveMacros = mutableListOf<Command>()

    private var phiTrim = 0.0.deg;
    private var veloTrim = 0;
    private var hoodTrim = 0.0;

    var speedFactorDrive = 1.0;
    var speedFactorIntake = 1.0;
    var shootTransferSpeedFactor = 1.0;
    var lowerOverridePower = 0.0;

    var ofsX = 0.0;
    var ofsY = 0.0;
    var ofsH = 0.0.rad;

    override fun onStartButtonPressed() {
        TurretPhiSubsystem.started = true;
//        val file = File(Lefile.filePath)
//        val content = file.readText().split("\n")
//        val startX = content[0].toDouble()
//        val startY = content[1].toDouble()
//        val startH = content[2].toDouble()
//
////        PedroComponent.follower.pose = Pose(72.0, 72.0, -PI / 2)
//        PedroComponent.follower.pose = Pose(startX, startY, startH)

        MagblockServoSubsystem.unblock();
        MagblockServoSubsystem.block()
//        MagblockServoSubsystem.unblock()
//        TurretThetaSubsystem.SetThetaPos(0.63)()

//        gamepad1.setLedColor(0.0, 0.0, 255.0, -1)
//        gamepad2.setLedColor(255.0, 0.0, 0.0, -1)

//        val mecanum = MecanumDriverControlled(
//            lfw,
//            rfw,
//            lbw,
//            rbw,
//            -Gamepads.gamepad1.leftStickY,
//            Gamepads.gamepad1.leftStickX,
//            Gamepads.gamepad1.rightStickX,
//            FieldCentric {
//                if (isBlue) (h.inRad - PI).rad else h
//            }
//        )
//        mecanum();
        driverControlled!!()

        Gamepads.gamepad1.leftBumper whenBecomesTrue {
            val path = FollowPath(gateIntakeChain!!)
            activeDriveMacros.add(path)
            path()
            gamepad1.rumble(100)
        }
//        (if (isBlue) Gamepads.gamepad1.dpadLeft else Gamepads.gamepad1.dpadRight)
//            .whenBecomesTrue {
//                val path = FollowPath(farShootChain!!)
//                path()
//                activeDriveMacros.add(path)
//            }
//        (if (isBlue) Gamepads.gamepad1.dpadRight else Gamepads.gamepad1.dpadLeft)
//            .whenBecomesTrue {
//                val path = FollowPath(closeShootChain!!)
//                path()
//                activeDriveMacros.add(path)
//            }
//        Gamepads.gamepad1.leftBumper whenBecomesTrue {
//            val path = FollowPath(parkChain!!)
//            path()
//            activeDriveMacros.add(path)
//        }

        Gamepads.gamepad1.rightBumper whenBecomesTrue {
            speedFactorDrive = 0.5;
        } whenBecomesFalse {
            speedFactorDrive = 1.0;
        }

//        Gamepads.gamepad2.leftBumper whenBecomesTrue {
//            speedFactorIntake = 0.5;
//        } whenBecomesFalse {
//            speedFactorIntake = 1.0;
//        }

        val lowerMotorDrive = MagMotorSubsystem.DriverCommand(
            Gamepads.gamepad2.rightTrigger.map { it * speedFactorIntake },
            Gamepads.gamepad2.leftTrigger.map { it * speedFactorIntake },
            { lowerOverridePower }
        )
        lowerMotorDrive()

        val intakeMotorDrive = IntakeMotorSubsystem.DriverCommand(
            Gamepads.gamepad2.rightTrigger.map { it * speedFactorIntake },
            Gamepads.gamepad2.leftTrigger.map { it * speedFactorIntake },
            { lowerOverridePower }
        )
        intakeMotorDrive()

        Gamepads.gamepad2.rightTrigger.greaterThan(0.0) whenBecomesTrue {
            MagblockServoSubsystem.block();
            (InstantCommand { lowerOverridePower = 0.001 })();
            SequentialGroup(
//                Delay(0.33),
                InstantCommand { lowerOverridePower = 0.0 }
            )()
        }
//        whenBecomesFalse MagblockServoSubsystem.unblock


//        val magServoDrive = MagServoSubsystem.DriverCommandDefaultOn(
//            Gamepads.gamepad1.leftTrigger.greaterThan(0.0)
//        )
//        magServoDrive();
//        Gamepads.gamepad1.leftTrigger.greaterThan(0.0) whenBecomesTrue MagServoSubsystem.reverse
//        Gamepads.gamepad1.rightTrigger.greaterThan(0.0) whenBecomesTrue MagServoSubsystem.run

        Gamepads.gamepad1.rightTrigger.greaterThan(0.1) whenBecomesTrue ParallelGroup(
            MagblockServoSubsystem.unblock,
            SequentialGroup(
                Delay(0.5),
                InstantCommand {
                    lowerOverridePower = 1.0;
                    ShooterSubsystem.isShooting = true;
                },
            )
        ) whenBecomesFalse {
            lowerOverridePower = 0.0;
            MagblockServoSubsystem.block()
            ShooterSubsystem.isShooting = false
        }

        // notify d1
//        Gamepads.gamepad2.dpadUp whenBecomesTrue { gamepad1.rumble(450) }

        // manual mode toggle
//        Gamepads.gamepad2.leftBumper and Gamepads.gamepad2.triangle whenBecomesTrue {
//            autoAimEnabled = !autoAimEnabled;
//            gamepad2.rumble(450);
//        }

//        Gamepads.gamepad2.square whenFalse {
//            if (resetMode) {
//                TurretPhiSubsystem.SetTargetPhi(resetModePhiAngle, phiTrim).requires(TurretPhiSubsystem)()
//            } else if (autoAimEnabled) {
//                TurretPhiSubsystem.AutoAim(
//                    dxp, dyp, h, phiTrim
//                )()
//            } else {
//                // manual
//            }
//        }

        // not trimming in reset mode
        // reset mode toggle
        Gamepads.gamepad2.leftBumper and Gamepads.gamepad2.rightBumper whenBecomesTrue {
            resetMode = !resetMode;
            if (resetMode) {
                // 180.0.deg corresponds to turret facing backwards
                gamepad2.rumble(200)
                gamepad2.setLedColor(255.0, 255.0, 0.0, -1)
                resetTypeHeading = false;
            } else {
                // reset position
                if (resetTypeHeading) {
                    PedroComponent.follower.pose = Pose(
                        PedroComponent.follower.pose.x,
                        PedroComponent.follower.pose.y,
                        resetModeParams.h.inRad
                    )
                } else {
                    PedroComponent.follower.pose = Pose(resetModeParams.x, resetModeParams.y, resetModeParams.h.inRad)
                }
//                PedroComponent.follower.pose = Pose(resetModeParams.x, resetModeParams.y, resetModeParams.h.inRad)
                gamepad2.rumble(200)
                gamepad2.setLedColor(255.0, 0.0, 0.0, -1)
            }
        }

        Gamepads.gamepad2.circle whenBecomesTrue {
            if (resetMode) {
                resetTypeHeading = true;
                gamepad2.setLedColor(0.0, 255.0, 0.0, -1)
            }
        }
        Gamepads.gamepad2.square whenBecomesTrue {
            if (resetMode) {
                resetTypeHeading = false;
                gamepad2.setLedColor(255.0, 255.0, 0.0, -1)
            }
        }

        // I think l/r only makes sense when robot facing away (approx same direction person is facing)
        Gamepads.gamepad2.dpadRight whenBecomesTrue {
            PedroComponent.follower.pose = Pose(
                PedroComponent.follower.pose.x,
                PedroComponent.follower.pose.y,
                PedroComponent.follower.pose.heading + Math.toRadians(1.0),
            )
//            phiTrim -= 2.0.deg
        }
        Gamepads.gamepad2.dpadLeft whenBecomesTrue {
            PedroComponent.follower.pose = Pose(
                PedroComponent.follower.pose.x,
                PedroComponent.follower.pose.y,
                PedroComponent.follower.pose.heading - Math.toRadians(1.0),
            )
//            phiTrim += 2.0.deg
        }

        Gamepads.gamepad2.rightBumper whenBecomesTrue {
            veloTrim += 10;
        }
        Gamepads.gamepad2.leftBumper whenBecomesTrue {
            veloTrim -= 10;
        }

        Gamepads.gamepad2.dpadUp whenBecomesTrue {
            hoodTrim += 0.025;
        }
        Gamepads.gamepad2.dpadDown whenBecomesTrue {
            hoodTrim -= 0.025;
        }

        Gamepads.gamepad2.triangle whenBecomesTrue {
            shootTransferSpeedFactor = min(shootTransferSpeedFactor + 0.1, 1.0)
        }
        Gamepads.gamepad2.cross whenBecomesTrue {
            shootTransferSpeedFactor = max(shootTransferSpeedFactor - 0.1, 0.0)
        }

        Gamepads.gamepad2.leftStickX lessThan -0.9 whenBecomesTrue {
            if (isBlue) {
                ofsY += 1;
            } else {
                ofsY -= 1;
            }
        }
        Gamepads.gamepad2.leftStickX greaterThan 0.9 whenBecomesTrue {
            if (isBlue) {
                ofsY -= 1;
            } else {
                ofsY += 1;
            }
        }
        Gamepads.gamepad2.leftStickY greaterThan 0.9 whenBecomesTrue {
            if (isBlue) {
                ofsX -= 1;
            } else {
                ofsX += 1;
            }
        }
        Gamepads.gamepad2.leftStickY lessThan -0.9 whenBecomesTrue {
            if (isBlue) {
                ofsX += 1;
            } else {
                ofsX -= 1;
            }
        }

        // reset trims
        Gamepads.gamepad2.rightStickButton whenBecomesTrue {
            ofsX = 0.0;
            ofsY = 0.0;
            phiTrim = 0.0.rad;
            veloTrim = 0;
            hoodTrim = 0.0;
            gamepad2.rumble(500);
        }
    }

    var lastRuntime = 0.0
    var dxyp = 0.0;
    var dxp = 0.0;
    var dyp = 0.0;
    override fun onUpdate() {
        telemetry.addLine("TRIMMING:")
//        telemetry.addData("PHI TRIM", abs(phiTrim.inDeg).toString() + " deg " + if (phiTrim.inDeg < 0.0) "RIGHT" else "LEFT");
        telemetry.addData("VELO TRIM", "$veloTrim tps");
        telemetry.addData("HOOD TRIM", abs(hoodTrim).toString() + " tk " + if (hoodTrim < 0.0) "FLATTER" else "CURVIER");
        telemetry.addData("X TRIM", ofsX);
        telemetry.addData("Y TRIM", ofsY)

        telemetry.addLine()
        telemetry.addData("SHOOT TRANSFER SPEED FACTOR", shootTransferSpeedFactor);

//        Runtime.getRuntime().gc()

        telemetry.addLine("==================")

        telemetry.addData("Memory Left", Runtime.getRuntime().maxMemory() -
                Runtime.getRuntime().totalMemory() +
                Runtime.getRuntime().freeMemory())
        telemetry.addData("Memory Used", Runtime.getRuntime().totalMemory() -
                Runtime.getRuntime().freeMemory())

        telemetry.addLine("==================")
        telemetry.addData("Loop Time (ms)", runtime - lastRuntime);
        lastRuntime = runtime;

//        cv!!.updateCV()
        PedroComponent.follower.update()
//        if (
//            cv!!.hasDetection()
////            && hypot(vx, vy) < 1.0  // todo: consider adding if cam bad during movement
//        ) {
//            PedroComponent.follower.pose = Pose(cv!!.x, cv!!.y, cv!!.h);
//        }
//        odom!!.updateOdom();

        if (
            activeDriveMacros.isNotEmpty() &&
            (
                    abs(Gamepads.gamepad1.leftStickX.get()) > 0.02 ||
                            abs(Gamepads.gamepad1.leftStickY.get()) > 0.02 ||
                            abs(Gamepads.gamepad1.rightStickX.get()) > 0.02
                    )
        ) {
            // untrigger macro
            activeDriveMacros.forEach {
                try {
                    it.stop(true)
                    CommandManager.cancelCommand(it)
                } finally {

                }
            }
            activeDriveMacros.clear()
            PedroComponent.follower.startTeleopDrive()
        }

        val dx = goalX - x
        val dy = goalY - y
        val dxy = hypot(dx, dy)
        if (vxOld.isEmpty() || vyOld.isEmpty()) {
            vxOld = listOf(0.0, 0.0, 0.0);
            vyOld = listOf(0.0, 0.0, 0.0);
        } else {
            vxOld = vxOld.slice(IntRange(1, vxOld.lastIndex)) + listOf((PedroComponent.follower.pose.x - lastPose.pose.x) / (runtime - lastTime));
            vyOld = vyOld.slice(IntRange(1, vyOld.lastIndex)) + listOf((PedroComponent.follower.pose.y - lastPose.pose.y) / (runtime - lastTime));
        }
        vx = vxOld.average();
        vy = vyOld.average();
        if (axOld.isEmpty() || ayOld.isEmpty()) {
            axOld = listOf(0.0, 0.0, 0.0);
            ayOld = listOf(0.0, 0.0, 0.0);
        } else {
            axOld = axOld.slice(IntRange(1, axOld.lastIndex)) + listOf((vx - lastVX) / (runtime - lastTime));
            ayOld = ayOld.slice(IntRange(1, ayOld.lastIndex)) + listOf((vy - lastVY) / (runtime - lastTime));
        }
        val ax = axOld.average();
        val ay = ayOld.average();
        val timeFactor = (if (y < BORD_Y) distanceToTimeFar(dxy) else distanceToTimeClose(dxy))
        val dxp = { accelFactor: Double -> dx - 1.0 * vx * timeFactor - accelFactor * ax * timeFactor * timeFactor }
        val dyp = { accelFactor: Double -> dy - 1.0 * vy * timeFactor - accelFactor * ay * timeFactor * timeFactor }
        val dxyp = { accelFactor: Double -> hypot(dxp(accelFactor), dyp(accelFactor)) }
        val hp = h;

        PanelsTelemetry.telemetry.addData("vx", vx);
        PanelsTelemetry.telemetry.addData("vy", vy);
        PanelsTelemetry.telemetry.addData("hp", hp);
        PanelsTelemetry.telemetry.addData("dxp", dxp);
        PanelsTelemetry.telemetry.addData("dyp", dyp);
        lastPose = PedroComponent.follower.pose;
        lastVX = vx;
        lastVY = vy;
        lastTime = runtime;

        val rp = min(abs(hypot(vx,vy)) / 16.0, 1.0)
        gamepad2.rumble(rp * (1 - y / 144.0), rp * (y / 144.0), 100)


//        gamepad1.rumble((hypot(vx, vy) / 12.0 * 100.0).toInt())
//        gamepad2.rumble((hypot(vx, vy) / 12.0 * 100.0).toInt())

        PanelsTelemetry.telemetry.addData("RUNTIME", runtime);
        PanelsTelemetry.telemetry.addData("SHOOTING?", ShooterSubsystem.isShooting);
        PanelsTelemetry.telemetry.addData("DISTANCE TO GOAL (in)", dxy);
        PanelsTelemetry.telemetry.addData("TIME-ADJ DISTANCE TO GOAL (in)", dxyp);

//        // funny little auto shoot i'm trying
//        // todo: experimental
//        if (
//            inTriangle(x, y, 6.0) > 0 &&
//            ShooterSubsystem.getError() < 55.5
//            && vh < 20.deg  // todo: super experimental
//            && hypot(ax, ay) < 24.0  // todo: super duper experimental
//        ) {
//            lowerOverridePower = 1.0
//        } else {
//            lowerOverridePower = 0.0
//        }

        if (resetMode) {
            ShooterSubsystem.AutoAim(
                dxy,
                { dist ->
                    (
                            if (y < BORD_Y)
                                distanceToVelocityFar(dist)
                            else
                                distanceToVelocityClose(dist)
                    ) + veloTrim
                }
            )()
            TurretThetaSubsystem.AutoAim(
                dxy,
                { dist ->
                    (
                            if (y < BORD_Y)
                                distAndVeloToThetaFar(dist, ShooterSubsystem.velocity)
                            else
                                distAndVeloToThetaClose(dist, ShooterSubsystem.velocity)
                    ) + hoodTrim
                },
            )()
            TurretPhiSubsystem.SetTargetPhi(resetModePhiAngle, phiTrim).requires(TurretPhiSubsystem)()
        } else if (autoAimEnabled) {
//            val sotmFactor = 1 - ((dxyp - dxy) / 10.0).coerceIn(0.0, 1.0);
//            val sotmFactor = if (
//                    (abs(Gamepads.gamepad1.leftStickX.get()) <= 0.02) &&
//                    (abs(Gamepads.gamepad1.leftStickY.get()) <= 0.02)
//            ) 0.0 else 1.0;
            val sotmFactor = 1.0;
            val dist = { accelFactor: Double -> dxyp(accelFactor) * sotmFactor + dxy * (1 - sotmFactor) }
            telemetry.addData("sotm factor", sotmFactor);

            if (inTriangle(x, y, 5.0) > 0) {
                gamepad1.rumble(1.0, 1.0, 100)
            }

            if (y < BORD_Y) {
                // far zone
//                if (inTriangle(x, y, 64.0) == 2) {
                    ShooterSubsystem.AutoAim(
                        dist(0.0), { dist -> distanceToVelocityFar(dist) + veloTrim }
                    )()
                    TurretThetaSubsystem.SetThetaPos(
                        distAndVeloToThetaFar(dist(0.0), ShooterSubsystem.velocity) + hoodTrim
                    )()
//                }
            } else {
                // close zone
//                if (inTriangle(x, y, 64.0) == 1) {
                    ShooterSubsystem.AutoAim(
                        dist(0.0), { dist -> distanceToVelocityClose(dist) + veloTrim }
                    )()
                    TurretThetaSubsystem.SetThetaPos(
                        distAndVeloToThetaClose(dist(0.0), ShooterSubsystem.velocity) + hoodTrim
                    )()
//                }
            }
            TurretPhiSubsystem.AutoAim(
                dxp(0.0) * sotmFactor + dx * (1 - sotmFactor),
                dyp(0.0) * sotmFactor + dy * (1 - sotmFactor),
                hp, phiTrim
            )()
        } else {
            //ShooterSubsystem.Manual(

            //)
        }

        telemetry.addData("x (inch)", x);
        telemetry.addData("y (inch)", y);
        telemetry.addData("h (radians)", h);
        telemetry.addData(
            "distanceToGoal",
            hypot((goalX - x), (goalY - y))
        );
        telemetry.addData("ShooterSpeed", ShooterSubsystem.velocity);
        telemetry.addData("Angle", shootAngleVal.deg);
        telemetry.addData("targetPhi", TurretPhiSubsystem.targetPhi)
        telemetry.addData("inTriangle", inTriangle(x, y, 5.0))
        telemetry.update()

//        PanelsTelemetry.telemetry.addData("Vx (in/s)", vx)
//        PanelsTelemetry.telemetry.addData("Vy (in/s)", vy)
        PanelsTelemetry.telemetry.addData("cmd snp", CommandManager.snapshot)
        PanelsTelemetry.telemetry.update()
    }
    override fun onStop() {
        val file = File(Lefile.filePath)
        file.delete()
        file.createNewFile()
        while (!file.canWrite()) {}
        file.writeText(
            x.toString() + "\n" +
                    y.toString() + "\n" +
                    h.inRad.toString() + "\n"
        )
    }

    //    double[] x_top = {72,-8,152};
    //    double[] y_top = {64,144,144};
    //    double[] x_bottom = {72,40,104};
    //    double[] y_bottom = {32,0,0};
    fun inTriangle(x1: Double, y1: Double, margin: Double): Int {
        /**0 = none, 1 = top, 2 = bottom, -1 = error */
        if (x1 > 144 || x1 < 0 || y1 > 144 || y1 < 0) {
            return -1
        }

        // T triangle: vertices (72,64), (-8,144), (152,144)
        val inTop = (y1 >= -x1 + 144 - margin) && (y1 >= x1 - margin)

        // B triangle:  (40,0), (72,32), (104,0)
        val inBottom = (y1 <= x1 - (48 - margin)) && (y1 <= -x1 + 96 + margin)

        if (inBottom) return 2
        if (inTop) return 1
        return 0
    }
}