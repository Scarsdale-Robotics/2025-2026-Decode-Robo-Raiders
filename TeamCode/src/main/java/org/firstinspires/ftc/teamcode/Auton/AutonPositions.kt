package org.firstinspires.ftc.teamcode.Auton

import com.pedropathing.geometry.Pose

object AutonPositions {

    //base positions are in blue coordinates

    val startPose = Pose(63.7, 8.503, Math.toRadians(90.0)) // Start Pose of our robot.
    val pushTeammatePose = Pose(55.7, 8.503, Math.toRadians(90.0)) // Start Pose of our robot.

//    val startPoseClose = Pose(33.222, 135.5422, Math.toRadians(270.0)) // Start Pose of our robot.
    val startPoseClose = Pose(17.1887, 115.3623, Math.toRadians(270.0))

    val startAutonControlPos = Pose(95.3, 58.5)

    //====Push Auton====
    val startPoseFarPush = Pose(63.714, 13.4167, Math.toRadians(-90.0))
    val postPushPose = Pose(50.0, 14.0, Math.toRadians(-90.0))

    val L2IntakePose = Pose(20.0, 60.8, Math.toRadians(-180.0))
    val L2IntakeControlPose = Pose(60.9, 65.3)

    val L1IntakePose = Pose(21.0, 84.3, Math.toRadians(-180.0))
    val L1IntakeControlPose = Pose(42.6, 84.2)

    val L3IntakePose = Pose(20.0, 35.7, Math.toRadians(-180.0))
    val L3IntakeControlPose = Pose(50.5, 38.8)

    val HPZPose = Pose(11.4, 14.3, Math.toRadians(-96.0))
    val HPZControlPose = Pose(13.4, 44.6)

    val pApark = Pose(52.5, 71.8)

    //====24 Specific Positions====
    val start24ShootPos = Pose(24.9, 105.6, Math.toRadians(270.0))
    val intake1Pos24 = Pose(29.5, 90.8, Math.toRadians(270.0))
    val intake2Pos24 = Pose(29.5, 66.0, Math.toRadians(270.0))
    val intake3Pos24 = Pose(29.5, 43.3, Math.toRadians(270.0))
    val robotPark24 = Pose(42.8, 10.9, Math.toRadians(180.0))

    //====CoOp Specific Positions====
    val startCoOpControlPos = Pose(91.6, 91.2)

    val coOpFirstStartGateOpen = Pose(17.0, 72.4, Math.toRadians(180.0))
    val coOpFirstGateOpenControlPos = Pose(27.1, 77.11, Math.toRadians(180.0))
    val coOpStartGateOpen = Pose(17.8, 63.1, Math.toRadians(180.0))
    val coOpStartGateOpenControlPos = Pose(41.2, 66.85)

    val CoOpCommonIntakeControlPos1 = Pose(-40.0, 85.0)
    val CoOpCommonIntakeControlPos2 = Pose(82.0, 32.0)
    val CoOpCommonIntake = Pose(12.0, 36.0, Math.toRadians(180.0)) // Intake Pos1
    //======CoOp Positions END=======

    val shootPoseClose =
        Pose(60.0, 76.6, Math.toRadians(188.0)) // Close Shoot Pose of our robot.
    val shootPoseFar =
        Pose(51.7, 15.9, Math.toRadians(180.0)) // Far Shoot Pose of our robot.
    val gateOpenPose =
        Pose(21.0, 66.0, Math.toRadians(125.0)) // Gate Open Pose of our robot.
    val gateOpenControlPos =
        Pose(56.7, 51.2)
    val gateOpenPoseFromFar =
        Pose(21.1, 58.0, Math.toRadians(140.0)) // Gate Open Pose of our robot.
    val gateAfterOpenPose = Pose(17.1, 53.0, Math.toRadians(125.0)) // Gate After Open Pose of our robot.
    val altAfterOpenPose = Pose(18.0, 54.4, Math.toRadians(180.0)) // Gate After Open Pose of our robot.
    val altAfterOpenControlPose = Pose(28.4, 57.7)
//=======
//        Pose(15.0, 56.0, Math.toRadians(140.0)) // Gate After Open Pose of our robot.
//>>>>>>> Stashed changes

    val parkPoseFull = Pose(37.5, 31.0, Math.toRadians(270.0))

    val commonIntakePos = Pose(14.0, 10.9, Math.toRadians(180.0))
    val commonIntakeControlPos = Pose(54.8, 36.7)

    val autonParkPose = Pose(50.0, 76.6, Math.toRadians(180.0))

    // Non-constant positions
    val intake3Pos = Pose(21.4, 36.0, Math.toRadians(180.0)) // Intake Pos1
    val intake3ControlPos = Pose(48.4, 32.0)

    val intake2Pos = Pose(19.0, 58.0, Math.toRadians(180.0)) // Intake Pos2
    val intake2ControlPos = Pose(61.7, 71.0)

    val intake1Pos = Pose(21.4, 84.0, Math.toRadians(180.0)) // Intake Pos3

    fun Blue(bluePose: Pose): Pose { return bluePose }
    fun Red(bluePose: Pose): Pose {
        return Pose(144.0 - bluePose.x, bluePose.y, Math.toRadians(blueRedConvertAngle(Math.toDegrees(bluePose.heading))))
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