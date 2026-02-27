//package org.firstinspires.ftc.teamcode.opmodes.teleop
//
//import com.bylazar.configurables.annotations.Configurable
//import com.bylazar.telemetry.PanelsTelemetry
//import com.pedropathing.geometry.BezierLine
//import com.pedropathing.geometry.Pose
//import com.pedropathing.paths.HeadingInterpolator
//import com.pedropathing.paths.PathChain
//import dev.nextftc.core.commands.Command
//import dev.nextftc.core.commands.CommandManager
//import dev.nextftc.core.commands.groups.ParallelGroup
//import dev.nextftc.core.components.BindingsComponent
//import dev.nextftc.core.components.SubsystemComponent
//import dev.nextftc.core.units.Angle
//import dev.nextftc.core.units.deg
//import dev.nextftc.core.units.rad
//import dev.nextftc.extensions.pedro.FollowPath
//import dev.nextftc.extensions.pedro.PedroComponent
//import dev.nextftc.extensions.pedro.PedroDriverControlled
//import dev.nextftc.ftc.Gamepads
//import dev.nextftc.ftc.NextFTCOpMode
//import dev.nextftc.ftc.components.BulkReadComponent
//import org.firstinspires.ftc.teamcode.Auton.AutonPositions
//import org.firstinspires.ftc.teamcode.Auton.AutonPositions.Pos
//import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleOp.Companion.shootAngleDegrees
//import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleOp.Companion.speed1
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants
//import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
//import org.firstinspires.ftc.teamcode.utils.Lefile
//import java.io.File
//import kotlin.math.abs
//import kotlin.math.hypot
//
//
//
//
//@Configurable
//open class TeleOpBaseWithLocal(
//    private val isBlue: Boolean,
//    private val goalX: Double,
//    private val goalY: Double,
//    private val resetModeParams: ResetModeParams,
//    private val resetModePhiAngle: Angle,
//    private val distanceToVelocity: (Double) -> Double,
//    private val distanceToTheta: (Double) -> Angle,
//    private val distanceToTime: (Double) -> Double
//): NextFTCOpMode() {
//
//    var local: LocalizationSubsystem? = null;
//
//    val x: Double get() = local!!.x
//
//    val y: Double get() = local!!.y
//
//    val h: Angle get() = local!!.h.rad
//
//    val vx: Double get() = local!!.vx
//
//    val vy: Double get() = local!!.vy
//
//    val vh: Angle get() = local!!.vh.rad
//
//
//    var gateIntakeChain: PathChain? = null;
//    var farShootChain: PathChain? = null;
//    var closeShootChain: PathChain? = null;
////    var closeIntakeChain: PathChain;
//    var driverControlled: PedroDriverControlled? = null;
////
//
//    init {
//        addComponents(
//            PedroComponent(Constants::createFollower),
//            BulkReadComponent,
//            BindingsComponent
//        )
//    }
//
//    override fun onInit() {
//        ShooterSubsystem.off()
//        MagMotorSubsystem.off()
////        MagServoSubsystem.stop()
//
//        driverControlled = PedroDriverControlled(
//            Gamepads.gamepad1.leftStickY.map { if (isBlue) it else -it },
//            Gamepads.gamepad1.leftStickX.map { if (isBlue) it else -it },
//            -Gamepads.gamepad1.rightStickX,
//            false
//        )
//
//        gateIntakeChain = PedroComponent.follower.pathBuilder()
//            .addPath(
//                BezierLine(
//                    PedroComponent.follower::getPose,
//                    Pos(AutonPositions.gateOpenPose, isBlue)
//                )
//            )
//            .setHeadingInterpolation(
//                HeadingInterpolator.linearFromPoint(
//                    PedroComponent.follower::getHeading,
//                    Pos(AutonPositions.gateOpenPose, isBlue).heading,
//                    0.75
//                )
//            )
//            .addPath(
//                BezierLine(
//                    Pos(AutonPositions.gateOpenPose, isBlue),
//                    Pos(AutonPositions.gateAfterOpenPose, isBlue)
//                )
//            )
//            .setLinearHeadingInterpolation(
//                Pos(AutonPositions.gateOpenPose, isBlue).heading,
//                Pos(AutonPositions.gateAfterOpenPose, isBlue).heading,
//                0.8
//            )
//            .build()
//
//        farShootChain = PedroComponent.follower.pathBuilder()
//            .addPath(
//                BezierLine(
//                    PedroComponent.follower::getPose,
//                    Pos(AutonPositions.shootPoseFar, isBlue)
//                )
//            )
//            .setTangentHeadingInterpolation()
////            .setHeadingInterpolation(
////                HeadingInterpolator.linearFromPoint(
////                    PedroComponent.follower::getHeading,
////                    Pos(AutonPositions.shootPoseFar, isBlue).heading,
////                    0.9
////                )
////            )
//            .build()
//
//        closeShootChain = PedroComponent.follower.pathBuilder()
//            .addPath(
//                BezierLine(
//                    PedroComponent.follower::getPose,
//                    Pos(AutonPositions.shootPoseClose, isBlue)
//                )
//            )
//            .setTangentHeadingInterpolation()
////            .setHeadingInterpolation(
////                HeadingInterpolator.linearFromPoint(
////                    PedroComponent.follower::getHeading,
////                    Pos(AutonPositions.shootPoseClose, isBlue).heading,
////                    0.9
////                )
////            )
//            .build()
//
//
//
//        val file = File(Lefile.filePath)
//        val content = file.readText().split("\n")
//        val startX = content[0].toDouble()
//        val startY = content[1].toDouble()
//        val startH = content[2].toDouble()
//
//        PedroComponent.follower.pose = Pose(startX, startY, startH)
//        local = LocalizationSubsystem(startX, startY, startH, hardwareMap);
//    }
//
//
//
//    var activeDriveMacros = mutableListOf<Command>()
//
//    var speedFactor = 1.0;
//
//    var speedFactorDrive = 1.0;
//    var speedFactorIntake = 1.0;
//    var lowerOverridePower = 0.0;
//
//    override fun onStartButtonPressed() {
//        MagblockServoSubsystem.block()
//
//        gamepad1.setLedColor(0.0, 0.0, 255.0, -1)
//        gamepad2.setLedColor(255.0, 0.0, 0.0, -1)
//
//        driverControlled!!()
//
//        Gamepads.gamepad1.dpadUp whenBecomesTrue {
//            val path = FollowPath(gateIntakeChain!!)
//            path()
//            activeDriveMacros.add(path)
//        }
//        (if (isBlue) Gamepads.gamepad1.dpadLeft else Gamepads.gamepad1.dpadRight)
//            .whenBecomesTrue {
//                val path = FollowPath(farShootChain!!)
//                path()
////                ParallelGroup(
////                    TurretPhiSubsystem.AutoAim(
////                        goalX - Pos(AutonPositions.shootPoseFar, isBlue).x,
////                        goalY - Pos(AutonPositions.shootPoseFar, isBlue).y,
////                        Pos(AutonPositions.shootPoseFar, isBlue).heading.rad
////                    ),
////                    path
////                )()
//                activeDriveMacros.add(path)
//            }
//        (if (isBlue) Gamepads.gamepad1.dpadRight else Gamepads.gamepad1.dpadLeft)
//            .whenBecomesTrue {
//                val path = FollowPath(closeShootChain!!)
//                path()
////                ParallelGroup(
////                    TurretPhiSubsystem.AutoAim(
////                        goalX - Pos(AutonPositions.shootPoseClose, isBlue).x,
////                        goalY - Pos(AutonPositions.shootPoseClose, isBlue).y,
////                        Pos(AutonPositions.shootPoseClose, isBlue).heading.rad
////                    ),
////                    path
////                )()
//                activeDriveMacros.add(path)
//            }
//
//        Gamepads.gamepad1.rightBumper whenBecomesTrue {
//            speedFactor = 0.5;
//        } whenBecomesFalse {
//            speedFactor = 1.0;
//        }
//
//
//
//
//
//
//
//
//        // not trimming in reset mode
//        // reset mode toggle
//
//        // I think l/r only makes sense when robot facing away (approx same direction person is facing)
//
//    }
//
//    var lastRuntime = 0.0
//    override fun onUpdate() {
//        telemetry.addData("Loop Time (ms)", runtime - lastRuntime);
//        lastRuntime = runtime;
//
//        local?.updateLocalization()
//        PedroComponent.follower.update()
//        PedroComponent.follower.pose = getPoseFromLocalization();
////        PedroComponent.follower.update() //maybe should update after not before
//
//
//        if (
//            activeDriveMacros.isNotEmpty() &&
//            (
//                    abs(gamepad1.left_stick_x) > 0.0 ||
//                    abs(gamepad1.left_stick_y) > 0.0 ||
//                    abs(gamepad1.right_stick_x) > 0.0
//            )
//        ) {
//            // untrigger macro
//            activeDriveMacros.forEach { CommandManager.cancelCommand(it) }
//            activeDriveMacros.clear()
//            PedroComponent.follower.startTeleopDrive()
//        }
//
//
//
//        telemetry.addData("x (inch)", x);
//        telemetry.addData("y (inch)", y);
//        telemetry.addData("h (radians)", h);
//        telemetry.addData("distanceToGoal", hypot((goalX - x), (goalY - y)));
//        telemetry.addData("ShooterSpeed", speed1);
//        telemetry.addData("Angle", shootAngleDegrees.deg);
//        telemetry.update()
//
////        PanelsTelemetry.telemetry.addData("Vx (in/s)", vx)
////        PanelsTelemetry.telemetry.addData("Vy (in/s)", vy)
//        PanelsTelemetry.telemetry.addData("CMD", CommandManager.snapshot)
//        PanelsTelemetry.telemetry.update()
//    }
//
//    override fun onStop() {
//        val file = File(Lefile.filePath)
//        file.writeText(
//            x.toString() + "\n" +
//                    y.toString() + "\n" +
//                    h.inRad.toString() + "\n"
//        )
//    }
//
//    private fun getPoseFromLocalization(): Pose {
//        return Pose(
//            x,
//            y,
//            h.inRad
//        )
//    }
//}
