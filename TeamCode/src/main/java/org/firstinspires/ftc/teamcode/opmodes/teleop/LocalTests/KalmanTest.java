package org.firstinspires.ftc.teamcode.opmodes.teleop.LocalTests;

import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.TelemetryPluginConfig;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;

import java.util.List;

import kotlin.Unit;

public class KalmanTest extends LinearOpMode {
    LocalizationSubsystem local;

    boolean circle;
    boolean square;

    TelemetryManager panelsManager;

    @Override
    public void runOpMode() throws InterruptedException {
        local = new LocalizationSubsystem(72,72,(-Math.PI/2), hardwareMap); //starting pose
        circle = true;

        panelsManager = new TelemetryManager(
                () -> new TelemetryPluginConfig(), //defualt config
                (List<String> list) -> Unit.INSTANCE,
                (Long interval) -> Unit.INSTANCE
        );


        waitForStart();
        local = new LocalizationSubsystem(72,72,(-Math.PI/2), hardwareMap); //starting pose
        while(opModeIsActive()){
            local.updateLocalization();

            panelsManager.addLine("LOCALIZATION");
            panelsManager.addData("xl: ", local.getX());
            panelsManager.addData("yl: ", local.getY());
            panelsManager.addData("hl: ", local.getH());

            panelsManager.addData("Vxl: ", local.getVX());
            panelsManager.addData("Vyl: ", local.getVY());
            panelsManager.addData("Vhl: ", local.getVH());

            panelsManager.addData("Axl: ", local.getAX());
            panelsManager.addData("Ayl: ", local.getAY());
            panelsManager.addData("Ahl: ", local.getAH());

            panelsManager.addData("Kalmain gain x", local.getKalmangainX());
            panelsManager.addData("Kalmain gain y", local.getKalmangainY());
            panelsManager.addData("Kalmain gain h", local.getKalmangianH());

            panelsManager.update(telemetry);


        }


            if(circle && gamepad1.circle){
                local.resetLocalizationFromCamera();
                circle = !circle;
            }

            if(local.isInTopShootingZone()){
                panelsManager.addLine("In top shooting zone");
            }else if(local.isInBottomTriangle()){
                panelsManager.addLine("In bottom shooting zone");
            }else{
                panelsManager.addLine("In no shooting zone");
            }

    }
}
