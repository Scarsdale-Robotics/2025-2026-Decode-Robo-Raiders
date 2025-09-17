package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.TurretSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "NextFTC TeleOp Program Java")
public class TemplateTeleOp extends NextFTCOpMode {

  private final MotorEx frontLeftMotor = new MotorEx("front_left").reversed();
  private final MotorEx frontRightMotor = new MotorEx("front_right");
  private final MotorEx backLeftMotor = new MotorEx("back_left").reversed();
  private final MotorEx backRightMotor = new MotorEx("back_right");


  private boolean manualOverride = false;
  public TemplateTeleOp(){
    addComponents(
      new SubsystemComponent(CVSubsystem.INSTANCE, IntakeSubsystem.INSTANCE, LocalizationSubsystem.INSTANCE, OuttakeSubsystem.INSTANCE)
    );

  }

  @Override
  public void onStartButtonPressed() {
    Command driverControlled = new MecanumDriverControlled(
      frontLeftMotor,
      frontRightMotor,
      backLeftMotor,
      backRightMotor,
      Gamepads.gamepad1().leftStickY(),
      Gamepads.gamepad1().leftStickX(),
      Gamepads.gamepad1().rightStickX()
    );
    driverControlled.schedule();


    Gamepads.gamepad1().a()
      .whenBecomesTrue(OuttakeSubsystem.INSTANCE.openShooter)
      .whenBecomesFalse(OuttakeSubsystem.INSTANCE.closeShooter);


    ////////////
    // Intake //
    ////////////
    Gamepads.gamepad1().leftBumper()
      .whenBecomesTrue(IntakeSubsystem.INSTANCE.on);
    Gamepads.gamepad1().rightBumper()
      .whenBecomesTrue(IntakeSubsystem.INSTANCE.off);


    /////////////////////
    // Manual Override //
    /////////////////////

    Gamepads.gamepad2().leftStickButton()
      .whenBecomesTrue(OuttakeSubsystem.INSTANCE.toggleManualOverride);

  }

}
