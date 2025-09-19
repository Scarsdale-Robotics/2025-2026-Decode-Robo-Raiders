package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.TurretSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class OuttakeSubsystem extends SubsystemGroup {

  public static final OuttakeSubsystem INSTANCE = new OuttakeSubsystem();
  public final ShooterSubsystem shooterSubsystem = ShooterSubsystem.INSTANCE;
  public final TurretSubsystem turretSubsystem = TurretSubsystem.INSTANCE;

  private OuttakeSubsystem() {
    super(
      ShooterSubsystem.INSTANCE,
      TurretSubsystem.INSTANCE
    );
  }
}
