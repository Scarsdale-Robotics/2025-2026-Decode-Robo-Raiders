package org.firstinspires.ftc.teamcode.opmodes.teleop.LocalTests;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.TelemetryPluginConfig;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem;

import java.util.List;

import kotlin.Unit;

@TeleOp(name = "RangeTest")
@Configurable
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

            int check = this.inTriangle(odom.getROx1(), odom.getROy1());
            if (check == 0){panelsManager.addLine("None");}
            if (check == 1){panelsManager.addLine("Top");}
            if (check == 2){panelsManager.addLine("Bottom");}
            if (check == 3){panelsManager.addLine("Error");}

            panelsManager.update();
        }


    }

//    double[] x_top = {72,-8,152};
//    double[] y_top = {64,144,144};
//    double[] x_bottom = {72,40,104};
//    double[] y_bottom = {32,0,0};




    public int inTriangle(double x1, double y1){
        int MARGIN = 8;
        ///0 = none, 1 = top, 2 = bottom, 3 = error
        if (x1 > 144 || x1 < 0 || y1 > 144 || y1 < 0) {
            return 3;
        }

        // T triangle: vertices (72,64), (-8,144), (152,144)
        boolean inTop = (y1 <= -x1 + 144 - MARGIN) && (y1 <= x1 - MARGIN);

        // B triangle:  (40,0), (72,32), (104,0)
        boolean inBottom = (y1 <= x1 - (48 - MARGIN)) && (y1 <= -x1 + 96 + MARGIN);

        if (inTop)    return 1;
        if (inBottom) return 2;
        return 0;
    }
}
