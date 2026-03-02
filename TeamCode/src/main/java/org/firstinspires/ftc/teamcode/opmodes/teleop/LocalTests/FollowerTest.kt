package org.firstinspires.ftc.teamcode.opmodes.teleop.LocalTests

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.Auton.AutonPositions
import org.firstinspires.ftc.teamcode.Auton.AutonPositions.Pos
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem
import kotlin.math.abs

@TeleOp(name = "FollowerTest")
@Configurable
class FollowerTest: NextFTCOpMode() {

    var local: LocalizationSubsystem? = null;
    var driverControlled: PedroDriverControlled? = null;
    var gateIntakeChain: PathChain? = null;
    var activeDriveMacros = mutableListOf<Command>()


    val x: Double get() = local!!.x

    val y: Double get() = local!!.y

    val h: Angle get() = local!!.h.rad


    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        driverControlled = PedroDriverControlled(
            Gamepads.gamepad1.leftStickY.map {it},
            Gamepads.gamepad1.leftStickX.map {it},
            -Gamepads.gamepad1.rightStickX,
            false
        )

        gateIntakeChain = PedroComponent.follower.pathBuilder() //just to test follower its on blue side
            .addPath(
                BezierLine(
                    PedroComponent.follower::getPose,
                    Pos(AutonPositions.gateOpenPose, true)
                )
            )
            .setHeadingInterpolation(
                HeadingInterpolator.linearFromPoint(
                    PedroComponent.follower::getHeading,
                    Pos(AutonPositions.gateOpenPose, true).heading,
                    0.75
                )
            )
            .addPath(
                BezierLine(
                    Pos(AutonPositions.gateOpenPose, true),
                    Pos(AutonPositions.gateAfterOpenPose, true)
                )
            )
            .setLinearHeadingInterpolation(
                Pos(AutonPositions.gateOpenPose, true).heading,
                Pos(AutonPositions.gateAfterOpenPose, true).heading,
                0.8
            )
            .build()

        PedroComponent.follower.pose = Pose(72.0, 72.0, (-Math.PI/2))
        local = LocalizationSubsystem(72.0, 72.0, (-Math.PI/2), hardwareMap);

    }

    override fun onStartButtonPressed() {
        driverControlled!!()
        Gamepads.gamepad1.dpadUp whenBecomesTrue {
            val path = FollowPath(gateIntakeChain!!)
            path()
            activeDriveMacros.add(path)
        }
    }

    override fun onUpdate() {
        local?.updateLocalization()
        PedroComponent.follower.update()
        PedroComponent.follower.pose = getPoseFromLocalization();
//        PedroComponent.follower.update() //maybe should update after not before

        if (
            activeDriveMacros.isNotEmpty() &&
            (
                    abs(gamepad1.left_stick_x) > 0.0 ||
                            abs(gamepad1.left_stick_y) > 0.0 ||
                            abs(gamepad1.right_stick_x) > 0.0
                    )
        ) {
            // untrigger macro
            activeDriveMacros.forEach { CommandManager.cancelCommand(it) }
            activeDriveMacros.clear()
            PedroComponent.follower.startTeleopDrive()
        }
        this.UpdateTelemetry();

    }

    private fun UpdateTelemetry() {
        telemetry.addLine("Local")
        telemetry.addData("x (inch)", x);
        telemetry.addData("y (inch)", y);
        telemetry.addData("h (deg)", h.inDeg);
        telemetry.addData("Kalmain gain x", local!!.kalmangainX)
        telemetry.addData("Kalmain gain y", local!!.kalmangainY)
        telemetry.addData("Kalmain gain h", local!!.kalmangianH)
        telemetry.update()
    }

    private fun getPoseFromLocalization(): Pose {
        return Pose(x, y, h.inRad)
    }
}