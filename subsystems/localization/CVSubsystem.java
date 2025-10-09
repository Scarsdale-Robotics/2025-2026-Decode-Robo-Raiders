package org.firstinspires.ftc.teamcode.subsystems.localization;


import com.qualcomm.hardware.limelightvision.LLResult;
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

  public motif motif;
  public LLResult bin;

  public boolean side; //True = blue, False = red




  public CVSubsystem(double x1, double y1, double h, boolean side, HardwareMap hm){
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
    side = side;
    this.init();
  }

  public void init(){
    cam.start();
    imu.resetYaw();
  }

  public void closeCam(){
    cam.close();
    imu.resetYaw();
  }

  public motif getMotif() {
    cam.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
    LLResult llResult = cam.getLatestResult();
    if (llResult != null && llResult.isValid()) {
      int num = llResult.getFiducialResults().get(1).getFiducialId(); //check if its acc 1
      motif result = motif.FD(num);
      return result;
    }
    return motif.Na;
  }

  public enum motif {
    GPP(1, 21),
    PGP(2, 22),
    PPG(3, 23),
    Na(0, -1);
    private final int motifValue;
    private final int num;
    motif(int motifValue, int num) {this.motifValue = motifValue;this.num = num;}
    public int getMotifValue() {return motifValue;}
    public int getInputID() {return num;}
    public static motif FD(int num) {
      for (motif m : values()) {if (m.num == num) {return m;}}
      return Na;
    }
  }

  public void updateCV(){
    cam.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
    bin = cam.getLatestResult();
    if(bin.isValid() && bin != null){
      if(bin.getFiducialResults().get(1).getFiducialId() == 20 && side){ //blue


      }else if (bin.getFiducialResults().get(1).getFiducialId() == 24 && !side){ //red

      }else{
        return;
      }
    }
  }


  public boolean getSide(){return side;}


}
