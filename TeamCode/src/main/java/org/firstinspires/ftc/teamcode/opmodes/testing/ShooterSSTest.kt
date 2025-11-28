//package org.firstinspires.ftc.teamcode.opmodes.testing
//
//import com.bylazar.configurables.annotations.Configurable
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import dev.nextftc.core.components.BindingsComponent
//import dev.nextftc.core.components.SubsystemComponent
//import dev.nextftc.ftc.NextFTCOpMode
//import dev.nextftc.ftc.components.BulkReadComponent
//import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
//
//@TeleOp(name = "Shooter SS Test", group = "Config")
//@Configurable
//class ShooterSSTest : NextFTCOpMode() {
//    companion object {
//        var speed = 1400.0;
//    }
//
//    init {
//        addComponents(
//            SubsystemComponent(
//                ShooterSubsystem
//            ),
//            BindingsComponent,
//            BulkReadComponent,
//        )
//    }
//
//    override fun onUpdate() {
//        super.onUpdate()
//    }
//}