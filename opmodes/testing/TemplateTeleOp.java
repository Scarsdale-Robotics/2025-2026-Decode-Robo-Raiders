package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.TurretSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "NextFTC TeleOp Program Java")
public class TemplateTeleOp extends NextFTCOpMode {

  // aliases because that name is really long
  OuttakeSubsystem Outtake_Subsystem = OuttakeSubsystem.INSTANCE;
  IntakeSubsystem Intake_Subsystem = IntakeSubsystem.INSTANCE;

  private final MotorEx frontLeftMotor = new MotorEx("front_left").reversed();
  private final MotorEx frontRightMotor = new MotorEx("front_right");
  private final MotorEx backLeftMotor = new MotorEx("back_left").reversed();
  private final MotorEx backRightMotor = new MotorEx("back_right");



  public boolean manualOverrideEnabled;

  public TemplateTeleOp(){
    addComponents(
      new SubsystemComponent(IntakeSubsystem.INSTANCE, LocalizationSubsystem.INSTANCE, OuttakeSubsystem.INSTANCE)
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


    Gamepads.gamepad1().circle()
      .whenBecomesTrue(Outtake_Subsystem.stopShoot)
      .whenBecomesFalse(Outtake_Subsystem.shoot);


    ////////////
    // Intake //
    ////////////
    Gamepads.gamepad1().leftBumper()
      .whenBecomesTrue(Intake_Subsystem.in)
      .whenBecomesFalse(Intake_Subsystem.stop);
    Gamepads.gamepad1().rightBumper()
      .whenBecomesTrue(Intake_Subsystem.out)
      .whenBecomesFalse(Intake_Subsystem.stop);

    manualOverrideEnabled = false;


    /////////////////////
    // Manual Override //
    /////////////////////

    Gamepads.gamepad2().leftStickButton()
      .whenBecomesTrue(this.toggleManualOverride);

  }
  @Override public void onUpdate(){
    OuttakeSubsystem.INSTANCE.onUpdate();
    LocalizationSubsystem.INSTANCE.onUpdate();
    if(this.manualOverrideEnabled){
      // put manual turn code here
    }
    else{
      autoAim();
    }

  }



  // to Nathan: Implement later maybe? this is for auto targeting
  public void autoAim(){

  }

  ///////////////////////
  /// turret commands ///
  ///////////////////////


  public Command toggleManualOverride = new LambdaCommand()
    .setStart(() -> manualOverrideEnabled = !manualOverrideEnabled)
    .setIsDone(() -> true)
    .setRequirements(TurretSubsystem.INSTANCE);

}
