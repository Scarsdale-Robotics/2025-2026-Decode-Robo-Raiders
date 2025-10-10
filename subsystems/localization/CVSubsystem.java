package org.firstinspires.ftc.teamcode.subsystems.localization;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class CVSubsystem{

  public Limelight3A cam;
  public IMU imu;

  public double RCx1;   // Inch
  public double RCy1;  // Inch
  public double RCh;  //normalized to [pi, -pi)



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
      int num = llResult.getFiducialResults().get(0).getFiducialId(); //check if its acc 0
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
      if(bin.getFiducialResults().get(0).getFiducialId() == 20 && side){ //blue
        Pose3D pos = bin.getBotpose_MT2();
        RCx1 = pos.getPosition().x;
        RCy1 = pos.getPosition().y;
        RCh = pos.getOrientation().getYaw(AngleUnit.RADIANS);
      }else if (bin.getFiducialResults().get(0).getFiducialId() == 24 && !side){ //red
        Pose3D pos = bin.getBotpose_MT2();
        RCx1 = pos.getPosition().x;
        RCy1 = pos.getPosition().y;
        RCh = pos.getOrientation().getYaw(AngleUnit.RADIANS);
      }else{
        return;
      }
    }
  }



  ///gets///
  public boolean getSide(){return side;}
  public double getRCx1(){return RCx1;}
  public double getRCh(){return RCh;}
  public double getRCy1(){return RCy1;}
  public LLResult camResult(){
    if(cam.getLatestResult().isValid() && cam.getLatestResult() != null){
      return cam.getLatestResult();}
    else{
      return null;
    }
  }
  public boolean camStatus(){return cam.isRunning();}




}
