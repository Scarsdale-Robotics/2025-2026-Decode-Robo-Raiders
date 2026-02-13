package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.TelemetryPluginConfig;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.localization.CVSubsystem_VisionPortal;

import java.util.List;

import kotlin.Unit;


@TeleOp(name = "CvTest")
@Configurable
public class CvTest extends LinearOpMode {

    CVSubsystem_VisionPortal cv;
    private TelemetryManager panelsManager;

    @Override
    public void runOpMode() throws InterruptedException {
        cv = new CVSubsystem_VisionPortal(0,0,Math.PI/2, hardwareMap); //starting pose
        panelsManager = new TelemetryManager(
                () -> new TelemetryPluginConfig(), //defualt config
                (List<String> list) -> Unit.INSTANCE,
                (Long interval) -> Unit.INSTANCE
        );

        waitForStart();
        while(opModeIsActive()){
            cv.updateCV();
            panelsManager.addLine("CV");
            panelsManager.addData("xc: ", cv.getRCx1());
            panelsManager.addData("yc: ", cv.getRCy1());
            panelsManager.addData("yc: ", cv.getRCh());
            panelsManager.update(telemetry);
        }
    }
}
