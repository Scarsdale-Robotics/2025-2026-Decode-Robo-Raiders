package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.PathChain
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
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
import org.firstinspires.ftc.teamcode.Auton.AutonPositions
import org.firstinspires.ftc.teamcode.Auton.AutonPositions.Pos
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleOp.Companion.shootAngleDegrees
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleOp.Companion.speed1
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.utils.Lefile
import java.io.File
import kotlin.math.hypot


data class ResetModeParams(val x: Double, val y: Double, val h: Angle)

@Configurable
open class TeleOpBase(
    private val isBlue: Boolean,
    private val goalX: Double,
    private val goalY: Double,
    private val resetModeParams: ResetModeParams,
    private val resetModePhiAngle: Angle,
    private val distanceToVelocity: (Double) -> Double,
    private val distanceToTheta: (Double) -> Angle,
    private val distanceToTime: (Double) -> Double
): NextFTCOpMode() {
    val x:  Double get() { return (PedroComponent.follower.pose.x);}
    val y:  Double get() { return (PedroComponent.follower.pose.y);}
    val h:  Angle  get() { return (PedroComponent.follower.pose.heading).rad;}
    val vx: Double get() { return (PedroComponent.follower.velocity.xComponent);}
    val vy: Double get() { return (PedroComponent.follower.velocity.yComponent);}
    val vh: Angle  get() { return (PedroComponent.follower.velocity.theta.rad);}

    var gateIntakeChain: PathChain;
    var farShootChain: PathChain;
    var closeShootChain: PathChain;
//    var closeIntakeChain: PathChain;
    var driverControlled: PedroDriverControlled;

    init {
        addComponents(
            SubsystemComponent(
                IntakeMotorSubsystem,
                MagMotorSubsystem,
                MagblockServoSubsystem,
                MagServoSubsystem,
                ShooterSubsystem,
                TurretPhiSubsystem,
                TurretThetaSubsystem
            ),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )

        gateIntakeChain = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    PedroComponent.follower::getPose,
                    Pos(AutonPositions.gateOpenPose, isBlue)
                )
            )
            .setHeadingInterpolation(
                HeadingInterpolator.linearFromPoint(
                    PedroComponent.follower::getHeading,
                    Pos(AutonPositions.gateOpenPose, isBlue).heading,
                    0.75
                )
            )
            .addPath(
                BezierLine(
                    Pos(AutonPositions.gateOpenPose, isBlue),
                    Pos(AutonPositions.gateAfterOpenPose, isBlue)
                )
            )
            .setLinearHeadingInterpolation(
                Pos(AutonPositions.gateOpenPose, isBlue).heading,
                Pos(AutonPositions.gateAfterOpenPose, isBlue).heading,
                0.8
            )
            .build()

        farShootChain = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    PedroComponent.follower::getPose,
                    Pos(AutonPositions.shootPoseFar, isBlue)
                )
            )
            .setHeadingInterpolation(
                HeadingInterpolator.linearFromPoint(
                    PedroComponent.follower::getHeading,
                    Pos(AutonPositions.shootPoseFar, isBlue).heading,
                    0.9
                )
            )
            .build()

        closeShootChain = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    PedroComponent.follower::getPose,
                    Pos(AutonPositions.shootPoseClose, isBlue)
                )
            )
            .setHeadingInterpolation(
                HeadingInterpolator.linearFromPoint(
                    PedroComponent.follower::getHeading,
                    Pos(AutonPositions.shootPoseClose, isBlue).heading,
                    0.9
                )
            )
            .build()

        driverControlled = PedroDriverControlled(
            Gamepads.gamepad1.leftStickY.map { if (isBlue) it else -it },
            Gamepads.gamepad1.leftStickX.map { if (isBlue) it else -it },
            -Gamepads.gamepad1.rightStickX,
            false
        )

        val file = File(Lefile.filePath)
        val content = file.readText().split("\n")
        val startX = content[0].toDouble()
        val startY = content[1].toDouble()
        val startH = content[2].toDouble()

        PedroComponent.follower.pose = Pose(startX, startY, startH)
    }

    override fun onInit() {
        ShooterSubsystem.off()
        MagMotorSubsystem.off()
        MagServoSubsystem.stop()
    }

    private var autoAimEnabled = true;
    private var resetMode = false;

    var activeDriveMacros = mutableListOf<Command>()

    private var phiTrim = 0.0.deg;
    var speedFactor = 1.0;

    override fun onStartButtonPressed() {
        MagblockServoSubsystem.block()

        gamepad1.setLedColor(0.0, 0.0, 255.0, -1)
        gamepad2.setLedColor(255.0, 0.0, 0.0, -1)

        driverControlled()

        Gamepads.gamepad1.dpadUp whenBecomesTrue {
            val path = FollowPath(gateIntakeChain)
            path()
            activeDriveMacros.add(path)
        }
        (if (isBlue) Gamepads.gamepad1.dpadLeft else Gamepads.gamepad1.dpadRight)
            .whenBecomesTrue {
                val path = FollowPath(farShootChain)
                ParallelGroup(
                    TurretPhiSubsystem.AutoAim(
                        goalX - Pos(AutonPositions.shootPoseFar, isBlue).x,
                        goalY - Pos(AutonPositions.shootPoseFar, isBlue).y,
                        Pos(AutonPositions.shootPoseFar, isBlue).heading.rad
                    ),
                    path
                )()
                activeDriveMacros.add(path)
            }
        (if (isBlue) Gamepads.gamepad1.dpadRight else Gamepads.gamepad1.dpadLeft)
            .whenBecomesTrue {
                val path = FollowPath(closeShootChain)
                ParallelGroup(
                    TurretPhiSubsystem.AutoAim(
                        goalX - Pos(AutonPositions.shootPoseClose, isBlue).x,
                        goalY - Pos(AutonPositions.shootPoseClose, isBlue).y,
                        Pos(AutonPositions.shootPoseClose, isBlue).heading.rad
                    ),
                    path
                )()
                activeDriveMacros.add(path)
            }

        Gamepads.gamepad1.rightBumper whenBecomesTrue {
            speedFactor = 0.5;
        } whenBecomesFalse {
            speedFactor = 1.0;
        }

        val lowerMotorDrive = MagMotorSubsystem.DriverCommand(
            Gamepads.gamepad2.rightTrigger,
            Gamepads.gamepad2.leftTrigger
        );
        lowerMotorDrive();

        val intakeMotorDrive = IntakeMotorSubsystem.DriverCommand(
            Gamepads.gamepad2.rightTrigger,
            Gamepads.gamepad2.leftTrigger
        )
        intakeMotorDrive()

//        val magServoDrive = MagServoSubsystem.DriverCommandDefaultOn(
//            Gamepads.gamepad1.leftTrigger.greaterThan(0.0)
//        )
//        magServoDrive();
        Gamepads.gamepad1.leftTrigger.greaterThan(0.0) whenBecomesTrue MagServoSubsystem.reverse
        Gamepads.gamepad1.rightTrigger.greaterThan(0.0) whenBecomesTrue MagServoSubsystem.run

        Gamepads.gamepad1.circle whenBecomesTrue {
            ParallelGroup(
                MagServoSubsystem.run,
                MagblockServoSubsystem.unblock
            )()
            ShooterSubsystem.isShooting = true  // todo: tell aaron to set this
        } whenBecomesFalse {
            ParallelGroup(
                MagServoSubsystem.stop,
                MagblockServoSubsystem.block
            )()
            ShooterSubsystem.isShooting = false
        }

        // manual mode toggle
        Gamepads.gamepad2.leftBumper and Gamepads.gamepad2.triangle whenBecomesTrue {
            autoAimEnabled = !autoAimEnabled;
            gamepad1.rumble(450);
            gamepad2.rumble(450);
        }

        Gamepads.gamepad2.circle whenTrue {
            val dx = goalX - x
            val dy = goalY - y
            val dxy = hypot(dx, dy)
            val dxp = dx + vx * distanceToTime(dxy)
            val dyp = dy + vy * distanceToTime(dxy)
            val dxyp = hypot(dxp, dyp)
            val hp = h + vh * distanceToTime(dxyp)
            TurretPhiSubsystem.AutoAim(
                dxp, dyp, hp, phiTrim
            )()
        }

        // not trimming in reset mode
        // reset mode toggle
        Gamepads.gamepad2.leftBumper and Gamepads.gamepad2.rightBumper whenBecomesTrue {
            resetMode = !resetMode;
            if (resetMode) {
                // 180.0.deg corresponds to turret facing backwards
                gamepad2.rumble(200)
                gamepad2.setLedColor(255.0, 255.0, 0.0, -1)
            } else {
                // reset position
                PedroComponent.follower.pose = Pose(resetModeParams.x, resetModeParams.y, resetModeParams.h.inRad)
                gamepad2.rumble(200)
                gamepad2.setLedColor(255.0, 0.0, 0.0, -1)
            }
        }
        // I think l/r only makes sense when robot facing away (approx same direction person is facing)
        Gamepads.gamepad2.dpadRight whenBecomesTrue {
            phiTrim -= 2.0.deg
        }
        Gamepads.gamepad2.dpadLeft whenBecomesTrue {
            phiTrim += 2.0.deg
        }
    }

    var lastRuntime = 0.0
    override fun onUpdate() {
        telemetry.addData("Loop Time (ms)", runtime - lastRuntime);
        lastRuntime = runtime;

        PedroComponent.follower.update()

        if (activeDriveMacros.isNotEmpty()) {
            // untrigger macro
            activeDriveMacros.forEach { CommandManager.cancelCommand(it) }
            activeDriveMacros.clear()
            driverControlled()
        }

        val dx = goalX - x
        val dy = goalY - y
        val dxy = hypot(dx, dy)
        val dxp = dx + vx * distanceToTime(dxy)
        val dyp = dy + vy * distanceToTime(dxy)
        val dxyp = hypot(dxp, dyp)
        val hp = h + vh * distanceToTime(dxyp)

        if (resetMode) {
            TurretPhiSubsystem.SetTargetPhi(resetModePhiAngle, phiTrim).requires(TurretPhiSubsystem)()
            ShooterSubsystem.AutoAim(
                dxyp,
                distanceToVelocity
            )()
            TurretThetaSubsystem.AutoAim(
                dxyp,
                distanceToTheta
            )()
        } else if (autoAimEnabled) {
            ShooterSubsystem.AutoAim(
                dxyp,
                distanceToVelocity
            )()
//            TurretPhiSubsystem.AutoAim(
//                dxp, dyp, hp, phiTrim
//            )()
            TurretThetaSubsystem.AutoAim(
                dxyp,
                distanceToTheta
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
        telemetry.addData("ShooterSpeed", speed1);
        telemetry.addData("Angle", shootAngleDegrees.deg);
        telemetry.update()

//        PanelsTelemetry.telemetry.addData("Vx (in/s)", vx)
//        PanelsTelemetry.telemetry.addData("Vy (in/s)", vy)
        PanelsTelemetry.telemetry.addData("CMD", CommandManager.snapshot)
        PanelsTelemetry.telemetry.update()
    }

    override fun onStop() {
        val file = File(Lefile.filePath)
        file.writeText(
            x.toString() + "\n" +
                    y.toString() + "\n" +
                    h.inRad.toString() + "\n"
        )
    }
}
