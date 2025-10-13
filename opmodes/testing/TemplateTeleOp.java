package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeControlledCommand;
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


  private final MotorEx frontLeftMotor = new MotorEx("front_left").reversed();
  private final MotorEx frontRightMotor = new MotorEx("front_right");
  private final MotorEx backLeftMotor = new MotorEx("back_left").reversed();
  private final MotorEx backRightMotor = new MotorEx("back_right");



  public boolean manualOverrideEnabled;

  public TemplateTeleOp(){
    addComponents(
      new SubsystemComponent(
              Robot.intakeSubsystem,
              Robot.outtakeSubsystem,
              Robot.localizationSubsystem
      )
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
      .whenBecomesTrue(ShooterSubsystem.INSTANCE.startShoot)
      .whenBecomesFalse(ShooterSubsystem.INSTANCE.stopShoot);


    ////////////
    // Intake //
    ////////////
    Gamepads.gamepad1().leftBumper()
      .whenBecomesTrue(IntakeControlledCommand.INSTANCE.in)
      .whenBecomesFalse(IntakeControlledCommand.INSTANCE.stop);
    Gamepads.gamepad1().rightBumper()
      .whenBecomesTrue(IntakeControlledCommand.INSTANCE.out)
      .whenBecomesFalse(IntakeControlledCommand.INSTANCE.stop);



    /////////////////////
    // Manual Override //
    /////////////////////
    manualOverrideEnabled = false;

    Gamepads.gamepad2().leftStickButton()
      .whenBecomesTrue(this.toggleManualOverride);

  }
  @Override public void onUpdate(){
    if(this.manualOverrideEnabled){
      // put manual turn code here
    }
    else{
      autoAim();
    }

  }



  ///////////////////////
  /// turret commands ///
  ///////////////////////


  public Command toggleManualOverride = new LambdaCommand()
    .setStart(() -> manualOverrideEnabled = !manualOverrideEnabled)
    .setIsDone(() -> true)
    .setRequirements(TurretSubsystem.INSTANCE);

  public void autoAim(){
    TurretSubsystem.INSTANCE.setAim(
            Math.toRadians(2),
            Math.toRadians(4)
    );
  }
}
