package org.firstinspires.ftc.teamcode.Auton

import com.pedropathing.geometry.Pose

object AutonPositions {

    //base positions are in blue coordinates

    val startPose = Pose(56.43, 8.503, Math.toRadians(180.0)) // Start Pose of our robot.
    val startPoseClose = Pose(26.8, 133.1, Math.toRadians(323.5)) // Start Pose of our robot.
    val startAutonControlPos = Pose(95.3, 58.5)

    val shootPoseClose =
        Pose(57.0, 76.6, Math.toRadians(180.0)) // Close Shoot Pose of our robot.
    val shootPoseFar =
        Pose(57.0, 13.5, Math.toRadians(180.0)) // Far Shoot Pose of our robot.
    val gateOpenPose =
        Pose(17.6, 60.5, Math.toRadians(125.0)) // Gate Open Pose of our robot.
    val gateAfterOpenPose = //F FTC MADE OUR MAIN STRATEGY ILLEGAL
        Pose(17.6, 54.0, Math.toRadians(125.0)) // Gate After Open Pose of our robot.

    val parkPoseFull = Pose(37.5, 31.0, Math.toRadians(270.0))

    val commonIntakePos = Pose(12.5, 10.9, Math.toRadians(180.0))
    val commonIntakeControlPos = Pose(54.8, 36.7)

    val autonParkPose = Pose(48.0, 76.6, Math.toRadians(180.0))

    // Non-constant positions
    val intake3Pos = Pose(21.4, 36.0, Math.toRadians(180.0)) // Intake Pos1
    val intake3ControlPos = Pose(48.4, 32.0)

    val intake2Pos = Pose(19.0, 58.0, Math.toRadians(180.0)) // Intake Pos2
    val intake2ControlPos = Pose(61.7, 71.0)

    val intake1Pos = Pose(21.5, 84.0, Math.toRadians(180.0)) // Intake Pos3

    fun Blue(bluePose: Pose): Pose { return bluePose }
    fun Red(bluePose: Pose): Pose {
        return Pose(144.0 - bluePose.x, bluePose.y, Math.toRadians(blueRedConvertAngle(bluePose.heading)))
    }
    fun Pos(bluePose: Pose, isBlue: Boolean): Pose {
        return if (isBlue) Blue(bluePose) else Red(bluePose);
    }

    fun blueRedConvertAngle (x : Double): Double {
        var newAngle = x
        if (x > 90 && x < 270) {
            newAngle = 180 - x
        }
        else {
            newAngle = 0 - x
            newAngle = 180 + newAngle
        }
        return newAngle
    }
}