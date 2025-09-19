package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.TurretSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class OuttakeSubsystem extends SubsystemGroup {
  public static final OuttakeSubsystem INSTANCE = new OuttakeSubsystem();
  private OuttakeSubsystem() {
    super(
      ShooterSubsystem.INSTANCE,
      TurretSubsystem.INSTANCE
    );
  }


  public void onUpdate(){
    TurretSubsystem.INSTANCE.onUpdate();
  }


  public void setAim(double theta, double phi) {
    TurretSubsystem.INSTANCE.setTheta(theta); TurretSubsystem.INSTANCE.setPhi(phi);
  }

  ////////////////////////
  /// shooter commands ///
  ////////////////////////
  public Command shoot = new LambdaCommand()
    .setStart(ShooterSubsystem.INSTANCE::startshoot)
    .setInterruptible(true)
    .setIsDone(()->false)
    .setRequirements(ShooterSubsystem.INSTANCE);
  public Command stopShoot = new LambdaCommand()
    .setStart(ShooterSubsystem.INSTANCE::stopshoot)
    .setIsDone(()->true)
    .setRequirements(ShooterSubsystem.INSTANCE);

}
