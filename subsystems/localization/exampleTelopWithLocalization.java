package org.firstinspires.ftc.teamcode.subsystems.localization;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;

import dev.nextftc.ftc.NextFTCOpMode;

public class exampleTelopWithLocalization extends NextFTCOpMode {

    LocalizationSubsystem localizationSubsystem;

    @Override
    public void onInit() {
        super.onInit();
        localizationSubsystem = new LocalizationSubsystem(3.0,4.0, Math.PI, true, hardwareMap);
        // at (3,4) inches - origin is (0,0) center of field
        // heading facing south
        // blue side
    }


    @Override
    public void onUpdate() {
        super.onUpdate();
        if()

    }

    public void telemetry(){
        telemetry.addData("x(inch)", localizationSubsystem.getX());
        telemetry.addData("y(inch)", localizationSubsystem.getY());
        telemetry.addData("heading(readians)", localizationSubsystem.getX());
        telemetry.addData("velocityX (")

    }
}
