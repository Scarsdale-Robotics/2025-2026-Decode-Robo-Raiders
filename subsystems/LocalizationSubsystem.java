package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.subsystems.localization.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem;

import dev.nextftc.core.subsystems.SubsystemGroup;

public class LocalizationSubsystem extends SubsystemGroup {
  public static final LocalizationSubsystem INSTANCE = new LocalizationSubsystem();
  private LocalizationSubsystem() {
    super(
      CVSubsystem.INSTANCE,
      OdometrySubsystem.INSTANCE
    );
  }


  public void onUpdate(){
    CVSubsystem.INSTANCE.onUpdate();
    OdometrySubsystem.INSTANCE.onUpdate();
  }
}
