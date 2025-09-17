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
    public boolean manualOverrideEnabled;

    public Command toggleManualOverride = new LambdaCommand()
      .setStart(() -> {
        manualOverrideEnabled = !manualOverrideEnabled;
      })
      .setIsDone(() -> true);


    public Command openShooter = new LambdaCommand()
      .setStart(()->{
        ShooterSubsystem.INSTANCE.setgoal(true);
      })
      .setUpdate(ShooterSubsystem.INSTANCE::runUpdate)
      .setIsDone(ShooterSubsystem.INSTANCE::isopen);
    public Command closeShooter = new LambdaCommand()
      .setStart(()->{
        ShooterSubsystem.INSTANCE.setgoal(false);
      })
      .setUpdate(ShooterSubsystem.INSTANCE::runUpdate)
      .setIsDone(() -> !ShooterSubsystem.INSTANCE.isopen());
}
