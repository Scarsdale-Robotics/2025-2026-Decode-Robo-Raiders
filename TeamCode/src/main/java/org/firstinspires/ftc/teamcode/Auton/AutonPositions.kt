package org.firstinspires.ftc.teamcode.Auton

import com.pedropathing.geometry.Pose
import dev.nextftc.core.units.Angle

object AutonPositions {

    //base positions are in blue coordinates

    val startPose = Pose(63.7, 8.503, Math.toRadians(90.0)) // Start Pose of our robot.
    val pushTeammatePose = Pose(55.7, 8.503, Math.toRadians(90.0)) // Start Pose of our robot.

//    val startPoseClose = Pose(33.222, 135.5422, Math.toRadians(270.0)) // Start Pose of our robot.
//    val startPoseClose = Pose(17.1887, 115.3623, Math.toRadians(270.0))
    val startPoseClose = Pose(15.0, 115.3623, Math.toRadians(270.0))

    val startAutonControlPos = Pose(95.3, 58.5)

    val farShootPoseCoOp = Pose(56.9, 12.7, Math.toRadians(180.0));
    val HPZCoOpPose = Pose(14.0, 9.4, Math.toRadians(180.0));
    val farParkCoOp = Pose(40.0, 17.1, Math.toRadians(180.0))

    //====Push Auton====
    val startPoseFarPush = Pose(63.714, 10.7, Math.toRadians(270.0))
    val postPushPose = Pose(50.0, 14.0, Math.toRadians(270.0))

    val L2IntakePose = Pose(21.67, 60.8-4.0, Math.toRadians(180.0))
    val L2IntakeControlPose = Pose(58.0, 65.3-4.0)

    val L1IntakePose = Pose(23.0, 84.3, Math.toRadians(180.0))
    val L1IntakeControlPose = Pose(42.6, 84.2)

    val L3IntakePose = Pose(21.0, 35.7, Math.toRadians(180.0))
    val L3IntakeControlPose = Pose(50.5, 38.8)

    val HPZPose = Pose(13.8, 13.6, Math.toRadians(263.0))
    val HPZControlPose = Pose(14.4, 45.6)

    val pApark = Pose(52.5, 65.8)

    //====24 Specific Positions====
    val start24ShootPos = Pose(24.9, 105.6, Math.toRadians(270.0))
    val intake1Pos24 = Pose(23.7, 90.8, Math.toRadians(270.0))
    val intake2Pos24 = Pose(23.7, 66.0, Math.toRadians(270.0))
    val intake3Pos24 = Pose(23.7, 43.3, Math.toRadians(270.0))
    val robotPark24 = Pose(42.8, 10.9, Math.toRadians(180.0))

    //====CoOp Specific Positions====
    val startCoOpControlPos = Pose(91.6, 91.2)

    val coOpFirstStartGateOpen = Pose(20.2, 76.4, Math.toRadians(180.0))
    val coOpFirstGateOpenControlPos = Pose(27.1, 77.11, Math.toRadians(180.0))
    val coOpStartGateOpen = Pose(19.0, 68.1, Math.toRadians(180.0))
    val coOpStartGateOpenControlPos = Pose(40.914, 58.286)
    val coOpGateToCommonControlPos = Pose(40.0, 53.7)

    val CoOpCommonIntakeControlPos1 = Pose(-40.0, 85.0)
    val CoOpCommonIntakeControlPos2 = Pose(82.0, 32.0)
//    val CoOpCommonIntake = Pose(15.0, 36.0, Math.toRadians(180.0)) // Intake Pos1
//    val CoOpCommonIntakeControlPos1 = Pose(40.0, 53.7)
//    val CoOpCommonIntakeControlPos2 = Pose(82.0, 32.0)
    val CoOpCommonIntake = Pose(12.0, 54.0, Math.toRadians(180.0)) // Intake Pos1
    val CoOpFinalCommonControl = Pose(32.977, 56.138);
    val CoOpFinalCommonIntake = Pose(15.0, 56.0, Math.toRadians(180.0))
    val CoOpFinalCommonToShootControl = Pose(39.714, 65.786)
    //======CoOp Positions END=======

    val shootPoseClose =
        Pose(56.2, 93.0, Math.toRadians(210.0)) // Close Shoot Pose of our robot.
    val shootPoseFar =
        Pose(51.7, 15.9, Math.toRadians(180.0)) // Far Shoot Pose of our robot.
    val gateOpenPose =
        Pose(14.8, 56.2, Math.toRadians(150.0)) // Gate Open Pose of our robot.
    val gateOpenPoseTele =
        Pose(14.8, 56.2, Math.toRadians(150.0)) // Gate Open Pose of our robot.
    val gateToShootControlPos =
        Pose(39.071, 69.029)
//    val altGateOpenPose =
//        Pose(21.0, 54.0, Math.toRadians(125.0)) // Gate Open Pose of our robot.
    val gateOpenControlPos =
        Pose(56.7, 51.2)
    val gateOpenPoseFromFar =
        Pose(21.1, 58.0, Math.toRadians(140.0)) // Gate Open Pose of our robot.
    val gateAfterOpenPose = Pose(13.8, 53.25, Math.toRadians(140.0)) // Gate After Open Pose of our robot.
    val altAfterOpenPose = Pose(18.0, 54.4, Math.toRadians(180.0)) // Gate After Open Pose of our robot.
    val altAfterOpenControlPose = Pose(28.4, 57.7)
//=======
//        Pose(15.0, 56.0, Math.toRadians(140.0)) // Gate After Open Pose of our robot.
//>>>>>>> Stashed changes

    val parkPoseFull = Pose(37.5, 31.0, Math.toRadians(270.0))

    val commonIntakePos = Pose(14.0, 10.9, Math.toRadians(180.0))
    val commonIntakeControlPos = Pose(54.8, 36.7)

    val autonParkPose = Pose(43.429, 80.886, Math.toRadians(225.0))

    // Non-constant positions
    val intake3Pos = Pose(21.4, 36.0, Math.toRadians(180.0)) // Intake Pos1
    val intake3ControlPos = Pose(55.4, 32.0)

    val intake2Pos = Pose(19.0, 58.0, Math.toRadians(180.0)) // Intake Pos2
    val intake2ControlPos = Pose(61.7, 71.0)

    val intake1Pos = Pose(21.4, 84.0, Math.toRadians(180.0)) // Intake Pos3

    fun Blue(bluePose: Pose): Pose { return bluePose }
    fun Blue(rads: Double): Double { return rads }
    fun Red(bluePose: Pose): Pose {
        return Pose(144.0 - bluePose.x, bluePose.y, Math.toRadians(blueRedConvertAngle(Math.toDegrees(bluePose.heading))))
    }
    fun Red(rads: Double): Double { return Math.toRadians(blueRedConvertAngle(Math.toDegrees(rads))) }

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