package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem

class FollowerTest: NextFTCOpMode() {
    var local: LocalizationSubsystem? = null;

    override fun onInit() {
        local = LocalizationSubsystem(0.0,0.0,0.0, hardwareMap);

    }



}