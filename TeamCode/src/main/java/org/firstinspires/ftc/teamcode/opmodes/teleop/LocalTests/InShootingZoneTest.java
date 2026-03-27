package org.firstinspires.ftc.teamcode.opmodes.teleop.LocalTests;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.TelemetryPluginConfig;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem;

import java.awt.*;
//import java.awt.Polygon;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import kotlin.Unit;

public class InShootingZoneTest extends LinearOpMode {
    OdometrySubsystem odom;
    private TelemetryManager panelsManager;



    @Override
    public void runOpMode() throws InterruptedException {
        odom = new OdometrySubsystem(72.0,72.0,(-Math.PI/2), hardwareMap); //starting pose
        panelsManager = new TelemetryManager(
                () -> new TelemetryPluginConfig(), //defualt config
                (List<String> list) -> Unit.INSTANCE,
                (Long interval) -> Unit.INSTANCE
        );
        waitForStart();
        odom.setPinpoint(72,72,(-Math.PI/2));//starting pose
        while(opModeIsActive()){
            odom.updateOdom();

            panelsManager.addLine("ODOM");
            panelsManager.addData("xo: ", odom.getROx1());
            panelsManager.addData("yo: ", odom.getROy1());
            panelsManager.addData("ho: ", odom.getROh());
        }


    }

    int[] x_top = {72,-8,152};
    int[] y_top = {64,144,144};
    int[] x_bottom = {72,40,104};
    int[] y_bottom = {32,0,0};



    Polygon top = new Polygon(x_top, y_top);
    Polygon 
    ///0 = none, 1 = top, 2 = bottom, 3 = error
    public int inTriangle(double x1, double y1){
        if(x1>144 || x1<0 || y1>144 || y1<0){
            return 0;
        }
        else if

        return 1;
    }
}
