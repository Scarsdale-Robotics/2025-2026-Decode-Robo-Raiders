package org.firstinspires.ftc.teamcode.subsystems.localization;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;

import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

public class exampleTelopWithLocalization extends NextFTCOpMode {

    LocalizationSubsystem localizationSubsystem;

    @Override
    public void onInit() {
        super.onInit();
        localizationSubsystem = new LocalizationSubsystem(3.0,4.0, Math.PI, true, hardwareMap);
        /// robot at (3,4) inches - origin is (0,0) center of field ///
        /// heading facing south ///
        /// blue side ///
    }


    @Override
    public void onUpdate() {
        super.onUpdate();

        this.telemetry();

        ///Make sure ur facing the motif before clicking circle otherwise it yells at you///
        boolean circle = true;
        if(Gamepads.gamepad1().circle().get() && circle){
            String motif = localizationSubsystem.getMotif().toString();
        }
        circle = true;
    }

    public void telemetry(){
        /// postion (inch)///
        telemetry.addData("x (inch): ", localizationSubsystem.getX());
        telemetry.addData("y (inch): ", localizationSubsystem.getY());
        telemetry.addData("h (radians): ", localizationSubsystem.getH());

        /// velocities (inch/ms)///
        telemetry.addData("Vx (inch/ms): ", localizationSubsystem.getVX());
        telemetry.addData("Vy (inch/ms): ", localizationSubsystem.getVY());
        telemetry.addData("Vh (rad/ms): ", localizationSubsystem.getVH());

        /// accelerations (inch/ms^2) ///
        telemetry.addData("Ax (inch/ms^2): ", localizationSubsystem.getAX());
        telemetry.addData("Ay (inch/ms^2): ", localizationSubsystem.getAY());
        telemetry.addData("Ah (rad/ms^2): ", localizationSubsystem.getAH());

        /// Time info ///
        telemetry.addData("Time since last update (ms): ", localizationSubsystem.getTimeSinceLastUpdate());
        telemetry.addData("Clock (ms): ", localizationSubsystem.getClock());
        telemetry.addData("Clock (s): ", localizationSubsystem.getTimeS());
        telemetry.addData("Last update time (ms): ", localizationSubsystem.getLastUpdateTime());

        /// Kalman ///
        telemetry.addData("Kalman Gain X: ", localizationSubsystem.getKalmangainX());
        telemetry.addData("Kalman Gain Y: ", localizationSubsystem.getKalmangainY());

        /// motif ///
        telemetry.addData("Detected Motif: ", localizationSubsystem.getMotif().toString());

        telemetry.update();

        telemetry.update();
    }
}
