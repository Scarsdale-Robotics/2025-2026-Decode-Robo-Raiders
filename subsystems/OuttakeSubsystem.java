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
    manualOverrideEnabled = false;
  }


  public void onUpdate(){
    ShooterSubsystem.INSTANCE.onUpdate();
    TurretSubsystem.INSTANCE.onUpdate();

  }
  public boolean manualOverrideEnabled;








  ///////////////////////
  /// turret commands ///
  ///////////////////////
  public Command runAutoTarget = new LambdaCommand()
    .setUpdate(TurretSubsystem.INSTANCE::autoAim)
    .setInterruptible(true)
    .setRequirements(TurretSubsystem.INSTANCE);


  public Command toggleManualOverride = new LambdaCommand()
    .setStart(() -> {
      manualOverrideEnabled = !manualOverrideEnabled;
      if (!manualOverrideEnabled)
        runAutoTarget.schedule();
    })
    .setIsDone(() -> manualOverrideEnabled)
    .setRequirements(TurretSubsystem.INSTANCE);






  ////////////////////////
  /// shooter commands ///
  ////////////////////////
  public Command shooterOn = new LambdaCommand()
    .setStart(()->ShooterSubsystem.INSTANCE.setOn(true))
    .setIsDone(()->true);
  public Command shooterOff = new LambdaCommand()
    .setStart(()->ShooterSubsystem.INSTANCE.setOn(false))
    .setIsDone(()->true);

}
