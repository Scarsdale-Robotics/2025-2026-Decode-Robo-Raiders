package org.firstinspires.ftc.teamcode.opmodes.teleop.LocalTests;



import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.TelemetryPluginConfig;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.localization.CVSubsystem_VisionPortal;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem;

import java.util.List;

import kotlin.Unit;

@TeleOp(name = "Local odom test")
@Configurable
public class BothTest extends LinearOpMode {

    CVSubsystem_VisionPortal cv;
    OdometrySubsystem odom;
    private TelemetryManager panelsManager;





    @Override
    public void runOpMode(){
        cv = new CVSubsystem_VisionPortal(72.0,72.0,(-Math.PI/2), hardwareMap); //starting pose
        odom = new OdometrySubsystem(72.0,72.0,(-Math.PI/2), hardwareMap); //starting pose
//


        panelsManager = new TelemetryManager(
                () -> new TelemetryPluginConfig(), //defualt config
                (List<String> list) -> Unit.INSTANCE,
                (Long interval) -> Unit.INSTANCE
        );



        waitForStart();
        odom.setPinpoint(72.0,72.0,(-Math.PI/2)); //starting pose
        while(opModeIsActive()){
            cv.updateCV();
            odom.updateOdom();

            panelsManager.addLine("ODOM");
            panelsManager.addData("xo: ", odom.getROx1());
            panelsManager.addData("yo: ", odom.getROy1());
            panelsManager.addData("ho: ", odom.getROh());

            panelsManager.addLine("CV");
            panelsManager.addData("xc: ", cv.getRCx1());
            panelsManager.addData("yc: ", cv.getRCy1());
            panelsManager.addData("yc: ", cv.getRCh());

            panelsManager.addLine("ERROR");
            panelsManager.addData("Xe: ", odom.getROx1() - cv.getRCx1());
            panelsManager.addData("Ye: ", odom.getROy1() - cv.getRCy1());


            panelsManager.update(telemetry);
        }
    }
}
