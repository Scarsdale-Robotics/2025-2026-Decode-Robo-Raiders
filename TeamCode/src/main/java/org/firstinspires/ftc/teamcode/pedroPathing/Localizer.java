package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Math.sqrt;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;

public class Localizer implements com.pedropathing.localization.Localizer {
    LocalizationSubsystem local;
    public Localizer(double x1, double y1, double h, HardwareMap hm){
        this.local = new LocalizationSubsystem(x1,y1,h, hm);
    }



    @Override
    public Pose getPose() {
       return new Pose(local.getX(), local.getY(), local.getH());
    }

    @Override
    public Pose getVelocity() {
        return new Pose(local.getVX(), local.getVY(), local.getVH());
    }

    @Override
    public Vector getVelocityVector() {
        return new Vector(Math.hypot(local.getVX(), local.getVY()), local.getVH());
    }

    @Override
    public void setStartPose(Pose setStart) {
        local.setPos(setStart.getX(), setStart.getY(), setStart.getHeading());
    }

    @Override
    public void setPose(Pose setPose) {
        local.setPos(setPose.getX(), setPose.getY(), setPose.getHeading());
    }

    @Override
    public void update() {
        local.updateLocalization();
    }

    @Override
    public double getTotalHeading() {
        return 0;
    }

    @Override
    public double getForwardMultiplier() {
        return 0;
    }

    @Override
    public double getLateralMultiplier() {
        return 0;
    }

    @Override
    public double getTurningMultiplier() {
        return 0;
    }

    @Override
    public void resetIMU() throws InterruptedException {
        local.setPos(this.getPose().getX(),this.getPose().getY(),0.0);
    }

    @Override
    public double getIMUHeading() {
        return local.getH();
    }

    @Override
    public boolean isNAN() {
        return false;
    }
}
