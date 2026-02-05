package org.firstinspires.ftc.teamcode.opmodes.teleop;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.localization.CVSubsystem_VisionPortal;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem;

@TeleOp(name = "Localtest")
public class Localtest extends LinearOpMode {

    CVSubsystem_VisionPortal cv;
    OdometrySubsystem odom;
    LocalizationSubsystem local;

    boolean circle;


    @Override
    public void runOpMode() throws InterruptedException {
        cv = new CVSubsystem_VisionPortal(0,0,Math.PI/2, hardwareMap);
        odom = new OdometrySubsystem(0,0,Math.PI/2, hardwareMap);
        local = new LocalizationSubsystem(0,0,Math.PI/2, hardwareMap);
        circle = true;

        waitForStart();
        while(opModeIsActive()){
            cv.updateCV();
            odom.updateOdom();
            local.updateLocalization();

            telemetry.addLine("ODOM");
            telemetry.addData("xo: ", odom.getROx1());
            telemetry.addData("yo: ", odom.getROy1());
            telemetry.addData("ho: ", odom.getROh());

            telemetry.addLine("CV");
            telemetry.addData("xc: ", cv.getRCx1());
            telemetry.addData("yc: ", cv.getRCy1());
            telemetry.addData("yc: ", cv.getRCh());

            telemetry.addLine("LOCALIZATION");
            telemetry.addData("xl: ", local.getX());
            telemetry.addData("yl: ", local.getY());
            telemetry.addData("hl: ", local.getH());

            telemetry.addData("Vxl: ", local.getVX());
            telemetry.addData("Vyl: ", local.getVY());
            telemetry.addData("Vhl: ", local.getVH());

            telemetry.addData("Axl: ", local.getAX());
            telemetry.addData("Ayl: ", local.getAY());
            telemetry.addData("Ahl: ", local.getAH());

            telemetry.update();

            if(circle && gamepad1.circle){
                local.resetLocalizationFromCamera();
                circle = !circle;
            }
        }
    }
}
