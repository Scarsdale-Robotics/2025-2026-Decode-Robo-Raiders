package org.firstinspires.ftc.teamcode.subsystems.localization;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import dev.nextftc.core.subsystems.Subsystem;

public class CVSubsystem{

  public Limelight3A cam;
  public IMU imu;

  public double RCx1;
  public double RCy1;
  public double RCh;



  public CVSubsystem(double x1, double y1, double h, HardwareMap hm){
    cam = hm.get(Limelight3A.class, "Limelight");
    cam.pipelineSwitch(0); //work on pipline
    //need limelight for tuning
    //make sure its in 3D :()
    imu = hm.get(IMU.class, "imu");
    RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD); // Tune
    imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    RCx1 = x1;
    RCy1 = y1;
    RCh = h;
  }

  public void init(){
    cam.start();
    imu.resetYaw();
  }

  public void closeCam(){
    cam.close();
    imu.resetYaw();
  }

  public void getMotif(){
    cam.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
    llresult
  }


}
