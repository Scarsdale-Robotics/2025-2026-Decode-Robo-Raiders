package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.IntakeControlledCommand;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;

public class Robot {
  public static IntakeControlledCommand intakeSubsystem = IntakeControlledCommand.INSTANCE;
  public static OuttakeSubsystem outtakeSubsystem = OuttakeSubsystem.INSTANCE;
  public static LocalizationSubsystem localizationSubsystem = LocalizationSubsystem.INSTANCE;
}
