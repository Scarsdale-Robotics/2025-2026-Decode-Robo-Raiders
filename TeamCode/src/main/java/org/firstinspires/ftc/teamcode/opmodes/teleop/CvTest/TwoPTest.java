package org.firstinspires.ftc.teamcode.opmodes.teleop.CvTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.cv.CvPortalManager;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.Mat;

import java.sql.Blob;
import java.util.List;

public class TwoPTest extends LinearOpMode {
    CvPortalManager Portal;
    Double x;
    Double y;
    Double h;
    List<ColorBlobLocatorProcessor.Blob> Blobs;

    Blob close;
    Blob big;
    @Override
    public void runOpMode() throws InterruptedException {
        this.Portal = new CvPortalManager(72,72,-Math.PI, true, hardwareMap);
        x = Portal.aprilTag.getX1();
    }
}
