package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.PathChain
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
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
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.BORD_Y
import org.firstinspires.ftc.teamcode.utils.Lefile
import java.io.File
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign


data class ResetModeParams(val x: Double, val y: Double, val h: Angle)

@Configurable
open class TeleOpBase(
    private val isBlue: Boolean,
    private val goalX: Double,
    private val goalY: Double,
    private val resetModeParams: ResetModeParams,
    private val resetModePhiAngle: Angle,
    private val distanceToVelocityClose: (Double) -> Double,
    private val distAndVeloToThetaClose: (Double, Double) -> Angle,
    private val distanceToVelocityFar: (Double) -> Double,
    private val distAndVeloToThetaFar: (Double, Double) -> Angle,
    private val distanceToTimeClose: (Double) -> Double,
    private val distanceToTimeFar: (Double) -> Double
): NextFTCOpMode() {
    val x:  Double get() { return (PedroComponent.follower.pose.x);}
    val y:  Double get() { return (PedroComponent.follower.pose.y);}
    val h:  Angle  get() { return (PedroComponent.follower.pose.heading).rad;}
    val vx: Double get() { return (PedroComponent.follower.velocity.xComponent);}
    val vy: Double get() { return (PedroComponent.follower.velocity.yComponent);}
    val vh: Angle  get() { return (PedroComponent.follower.velocity.theta.rad);}
    val ax: Double get() { return (PedroComponent.follower.acceleration.xComponent);}
    val ay: Double get() { return (PedroComponent.follower.acceleration.yComponent);}

    var gateIntakeChain: PathChain? = null;
    var farShootChain: PathChain? = null;
    var closeShootChain: PathChain? = null;
    //    var closeIntakeChain: PathChain;
    var driverControlled: PedroDriverControlled? = null;
    //    var driverControlled: PedroDriverControlled;
    var parkChain: PathChain? = null;

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

    var lockDirection = false;
    override fun onInit() {
        ShooterSubsystem.off()
        MagMotorSubsystem.off()
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
            Gamepads.gamepad1.leftStickY.deadZone(0.02).map { (if (isBlue) it else -it) * speedFactorDrive },
            Gamepads.gamepad1.leftStickX.deadZone(0.02).map { (if (isBlue) it else -it) * speedFactorDrive },
            -Gamepads.gamepad1.rightStickX.deadZone(0.02).map { it * speedFactorDrive },  // TODO: check if it * it is okay
            false
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
                    0.2
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
//            .setTangentHeadingInterpolation()
            .setHeadingInterpolation(
                HeadingInterpolator.linearFromPoint(
                    PedroComponent.follower::getHeading,
                    Pos(AutonPositions.shootPoseFar, isBlue).heading,
                    0.2
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
//            .setTangentHeadingInterpolation()
            .setHeadingInterpolation(
                HeadingInterpolator.linearFromPoint(
                    PedroComponent.follower::getHeading,
                    Pos(AutonPositions.shootPoseClose, isBlue).heading,
                    0.2
                )
            )
            .build()

        parkChain = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    PedroComponent.follower::getPose,
                    Pos(AutonPositions.parkPoseFull, isBlue)
                )
            )
//            .setTangentHeadingInterpolation()
            .setHeadingInterpolation(
                HeadingInterpolator.linearFromPoint(
                    PedroComponent.follower::getHeading,
                    Pos(AutonPositions.parkPoseFull, isBlue).heading,
                    1.0
                )
            )
            .build()

        val file = File(Lefile.filePath)
        val content = file.readText().split("\n")
        val startX = content[0].toDouble()
        val startY = content[1].toDouble()
        val startH = content[2].toDouble()

//        PedroComponent.follower.pose = Pose(72.0, 72.0, -PI / 2)
        PedroComponent.follower.pose = Pose(startX, startY, startH)
    }

    private var autoAimEnabled = true;
    private var resetMode = false;

    var activeDriveMacros = mutableListOf<Command>()

    private var phiTrim = 0.0.deg;
    private var veloTrim = 0;
    private var hoodTrim = 0.0.deg;

    var speedFactorDrive = 1.0;
    var speedFactorIntake = 1.0;
    var shootTransferSpeedFactor = 1.0;
    var lowerOverridePower = 0.0;

    //TODO: REMINDER TO CHECK ANTI-PEDRO BRANCH
    override fun onStartButtonPressed() {
        MagblockServoSubsystem.unblock()
        MagblockServoSubsystem.block()

        gamepad1.setLedColor(0.0, 0.0, 255.0, -1)
        gamepad2.setLedColor(255.0, 0.0, 0.0, -1)

        driverControlled!!()

//        Gamepads.gamepad1.dpadUp whenBecomesTrue {
//            val path = FollowPath(gateIntakeChain!!)
//            path()
//            activeDriveMacros.add(path)
//        }
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
        Gamepads.gamepad1.leftBumper whenBecomesTrue {
            val path = FollowPath(parkChain!!)
            path()
            activeDriveMacros.add(path)
        }

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
        );
        lowerMotorDrive();

        val intakeMotorDrive = IntakeMotorSubsystem.DriverCommand(
            Gamepads.gamepad2.rightTrigger.map { it * speedFactorIntake },
            Gamepads.gamepad2.leftTrigger.map { it * speedFactorIntake },
            { lowerOverridePower }
        )
        intakeMotorDrive()

//        val magServoDrive = MagServoSubsystem.DriverCommandDefaultOn(
//            Gamepads.gamepad1.leftTrigger.greaterThan(0.0)
//        )
//        magServoDrive();
//        Gamepads.gamepad1.leftTrigger.greaterThan(0.0) whenBecomesTrue MagServoSubsystem.reverse
//        Gamepads.gamepad1.rightTrigger.greaterThan(0.0) whenBecomesTrue MagServoSubsystem.run

        Gamepads.gamepad1.rightTrigger greaterThan 0.05 whenBecomesTrue {
            lowerOverridePower = if (y < BORD_Y) {
                0.8 * shootTransferSpeedFactor;
            } else {
                1.0 * shootTransferSpeedFactor;
            }
            MagblockServoSubsystem.unblock()
            ShooterSubsystem.isShooting = true  // todo: tell aaron to set this (nvm)
        } whenBecomesFalse {
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

        Gamepads.gamepad2.square whenFalse {
            if (resetMode) {
                TurretPhiSubsystem.SetTargetPhi(resetModePhiAngle, phiTrim).requires(TurretPhiSubsystem)()
            } else if (autoAimEnabled) {
                TurretPhiSubsystem.AutoAim(
                    dxp, dyp, h, phiTrim
                )()
            } else {
                // manual
            }
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

        Gamepads.gamepad2.rightBumper whenBecomesTrue {
            veloTrim += 10;
        }
        Gamepads.gamepad2.leftBumper whenBecomesTrue {
            veloTrim -= 10;
        }

        Gamepads.gamepad2.dpadUp whenBecomesTrue {
            hoodTrim += 0.5.deg;
        }
        Gamepads.gamepad2.dpadDown whenBecomesTrue {
            hoodTrim -= 0.5.deg;
        }

        Gamepads.gamepad2.triangle whenBecomesTrue {
            shootTransferSpeedFactor = min(shootTransferSpeedFactor + 0.1, 1.0)
        }
        Gamepads.gamepad2.cross whenBecomesTrue {
            shootTransferSpeedFactor = max(shootTransferSpeedFactor - 0.1, 0.0)
        }
    }

    var lastRuntime = 0.0
    var dxyp = 0.0;
    var dxp = 0.0;
    var dyp = 0.0;
    override fun onUpdate() {
        telemetry.addLine("TRIMMING:")
        telemetry.addData("PHI TRIM", abs(phiTrim.inDeg).toString() + " deg " + if (phiTrim.inDeg < 0.0) "RIGHT" else "LEFT");
        telemetry.addData("VELO TRIM", "$veloTrim tps");
        telemetry.addData("HOOD TRIM", abs(hoodTrim.inDeg).toString() + " deg " + if (hoodTrim.inDeg < 0.0) "FLATTER" else "CURVIER");

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

        PedroComponent.follower.update()

//        if (
//            activeDriveMacros.isNotEmpty() &&
//            (
//                    abs(gamepad1.left_stick_x) > 0.02 ||
//                            abs(gamepad1.left_stick_y) > 0.02 ||
//                            abs(gamepad1.right_stick_x) > 0.02
//                    )
//        ) {
//            // untrigger macro
//            activeDriveMacros.forEach { CommandManager.cancelCommand(it) }
//            activeDriveMacros.clear()
//            PedroComponent.follower.startTeleopDrive()
//        }

        val dx = goalX - x
        val dy = goalY - y
        val dxy = hypot(dx, dy)
        dxp = dx - (vx + 0.05 * ax) * (if (y < BORD_Y) distanceToTimeFar(dxy) else distanceToTimeClose(dxy))
        dyp = dy - (vy + 0.05 * ay) * (if (y < BORD_Y) distanceToTimeFar(dxy) else distanceToTimeClose(dxy))
        dxyp = hypot(dxp, dyp)

        PanelsTelemetry.telemetry.addData("RUNTIME", runtime);
        PanelsTelemetry.telemetry.addData("SHOOTING?", ShooterSubsystem.isShooting);
        PanelsTelemetry.telemetry.addData("DISTANCE TO GOAL (in)", dxy);
        PanelsTelemetry.telemetry.addData("TIME-ADJ DISTANCE TO GOAL (in)", dxyp);

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
        } else if (autoAimEnabled) {
            ShooterSubsystem.AutoAim(
                dxy * 0.8 + dxyp * 0.2,  // TODO: hope this is not sus
//                dxyp,
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
                dxyp,
                { dist ->
                    (
                            if (y < BORD_Y)
                                distAndVeloToThetaFar(dist, ShooterSubsystem.velocity)
                            else
                                distAndVeloToThetaClose(dist, ShooterSubsystem.velocity)
                    ) + hoodTrim
                },
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
        PanelsTelemetry.telemetry.addData("cmd snp", CommandManager.snapshot)
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