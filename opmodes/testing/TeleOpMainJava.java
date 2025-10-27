//package org.firstinspires.ftc.teamcode.opmodes.testing;
//
//import com.qualcomm.robotcore.hardware.Gamepad;
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.commands.utility.InstantCommand;
//import dev.nextftc.core.components.SubsystemComponent;
//
//import dev.nextftc.core.units.Angle;
//import dev.nextftc.ftc.Gamepads;
//import dev.nextftc.ftc.NextFTCOpMode;
//import dev.nextftc.hardware.driving.MecanumDriverControlled;
//import dev.nextftc.hardware.impl.MotorEx;
//import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.outtake.TurretSubsystem;
//
//public class TeleOpMainJava extends NextFTCOpMode {
//  private MotorEx intakeMotor = new MotorEx("intake");
//
//  private MotorEx frontLeftMotor = new MotorEx("frontLeft");
//  private MotorEx frontRightMotor = new MotorEx("frontRight");
//  private MotorEx backLeftMotor = new MotorEx("backLeft");
//  private MotorEx backRightMotor = new MotorEx("backRight");
//
//  private boolean autoAimEnabled = true;
//
//  private Command autoAimCommand;
//  private Command manualAimCommand;
//  private Angle angleByDistance(double dist) {
//    return Angle.fromDeg(0);
//  }
//  @Override
//  public final void onInit() {
//    addComponents(
//      new SubsystemComponent(
//        OuttakeSubsystem.INSTANCE,
//        LowerSubsystem.INSTANCE
//      )  // todo: consider add localization subsystem
//    );
//
//    autoAimCommand = new TurretSubsystem.AutoAim( ()->0.0 ,  ()->0.0 , (this::angleByDistance));  // todo: connect with localization
//    manualAimCommand = new TurretSubsystem.DriverCommand(
//      Gamepads.gamepad2().triangle(),
//      Gamepads.gamepad2().cross(),
//      Gamepads.gamepad2().dpadUp(),
//      Gamepads.gamepad2().dpadDown(),
//      Gamepads.gamepad2().dpadLeft(),
//      Gamepads.gamepad2().dpadRight(),
//      Gamepads.gamepad2().circle(),
//      Angle.fromDeg(0.0),
//      Angle.fromDeg(58.0),
//      Angle.fromDeg(30.0),
//      Angle.fromDeg(45.0)
//
//      );
//  }
//
//  public void onStartButtonPressed() {
//      new IntakeSubsystem.DriverCommandDefaultOn(
//      Gamepads.gamepad1().leftTrigger()
//      ).schedule();
//
//      new MecanumDriverControlled(
//        frontLeftMotor,
//        frontRightMotor,
//        backLeftMotor,
//        backRightMotor,
//        Gamepads.gamepad1().leftStickY(),
//        Gamepads.gamepad1().leftStickX(),
//        Gamepads.gamepad1().rightStickX()
//      ).schedule();
//
//    // auto-aim
//    autoAimCommand.schedule();
//
//    // circle --> shoot
//    Gamepads.gamepad1().circle().whenBecomesTrue(MagblockServoSubsystem.open());
//    Gamepads.gamepad1().circle().whenBecomesFalse(MagblockServoSubsystem.close());
//
//    // g2 both bumpers --> toggle autoAim
//    Gamepads.gamepad2().leftBumper().and(Gamepads.gamepad2().rightBumper()).whenBecomesTrue(new InstantCommand(()->
//    {
//      if (autoAimEnabled) {
//        autoAimCommand.cancel();
//        manualAimCommand.schedule();
//      } else {
//        autoAimCommand.schedule();
//        manualAimCommand.cancel();
//      }
//      autoAimEnabled = !autoAimEnabled;
//      gamepad1.rumble(0.5, 0.5, 200);
//      gamepad2.rumble(0.5, 0.5, 200);
//    }));
//
//    // g2 square --> try to clear balls, fix jam
//    Gamepads.gamepad2().square().whenBecomesTrue(LowerSubsystem.fixJam);
//
//    // todo: localization reset
//    // todo: auto aim test teleop
//  }
//
//  @Override
//  public void onUpdate() {
//    // indicators
//    if (!autoAimEnabled) {
//      gamepad2.setLedColor(
//        255.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
//    } else {
//      gamepad2.setLedColor(
//        255.0, 255.0, 255.0, Gamepad.LED_DURATION_CONTINUOUS);
//    }
//  }
//}
