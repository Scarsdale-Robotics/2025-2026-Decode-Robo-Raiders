package org.firstinspires.ftc.teamcode.opmodes.teleop;



import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.panels.*;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.TelemetryPluginConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.localization.CVSubsystem_VisionPortal;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem;

import java.util.List;

import kotlin.Unit;

@TeleOp(name = "Localtest")
public class Localtest extends LinearOpMode {

    CVSubsystem_VisionPortal cv;
    OdometrySubsystem odom;
    LocalizationSubsystem local;

    boolean circle;
    boolean square;
    private TelemetryManager panelsManager;


    Pose pose;



    @Override
    public void runOpMode() throws InterruptedException {
        cv = new CVSubsystem_VisionPortal(0,0,Math.PI/2, hardwareMap); //starting pose
        odom = new OdometrySubsystem(0,0,Math.PI/2, hardwareMap); //starting pose
        local = new LocalizationSubsystem(0,0,Math.PI/2, hardwareMap); //starting pose
        circle = true;


        panelsManager = new TelemetryManager(
                () -> new TelemetryPluginConfig(), //defualt config
                (List<String> list) -> Unit.INSTANCE,
                (Long interval) -> Unit.INSTANCE
        );



        waitForStart();
        while(opModeIsActive()){
            cv.updateCV();
            odom.updateOdom();
            local.updateLocalization();

            panelsManager.addLine("ODOM");
            panelsManager.addData("xo: ", odom.getROx1());
            panelsManager.addData("yo: ", odom.getROy1());
            panelsManager.addData("ho: ", odom.getROh());

            panelsManager.addLine("CV");
            panelsManager.addData("xc: ", cv.getRCx1());
            panelsManager.addData("yc: ", cv.getRCy1());
            panelsManager.addData("yc: ", cv.getRCh());

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

            panelsManager.update(telemetry);

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
}
