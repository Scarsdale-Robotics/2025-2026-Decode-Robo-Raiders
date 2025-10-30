package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;

public class LocalTest extends LinearOpMode {
    LocalizationSubsystem test;

    public LocalTest() {
        test = new LocalizationSubsystem(0,0,0,true,hardwareMap);
    }

    public void UpEtelemetry() {
        /** postion (inch)/// */
        telemetry.addData("x (inch): ", test.getX());
        telemetry.addData("y (inch): ", test.getY());
        telemetry.addData("h (radians): ", test.getH());
    }

    @Override
    public void runOpMode() throws InterruptedException {
        test.updateLocalization();
        this.UpEtelemetry();
    }
}
