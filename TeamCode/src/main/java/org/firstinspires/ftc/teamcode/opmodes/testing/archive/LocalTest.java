package org.firstinspires.ftc.teamcode.opmodes.testing.archive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "AutoTest", group = "Testing")
public class LocalTest extends NextFTCOpMode {

    private OdometrySubsystem odom;
    private Command driverControlled;



    private final MotorEx frontLeftMotor = new MotorEx("FL").reversed(); //2
    private final MotorEx frontRightMotor = new MotorEx("FR"); //0
    private final MotorEx backLeftMotor = new MotorEx("BL").reversed(); //3
    private final MotorEx backRightMotor = new MotorEx("BR"); //1


    public LocalTest() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY(), // forward/backward
                Gamepads.gamepad1().leftStickX(),          // strafe
                Gamepads.gamepad1().rightStickX()          // turn
        );
        driverControlled.schedule();

        //let it crash if it fails to init
        odom = new OdometrySubsystem(0, 0, 0, hardwareMap);
        telemetry.addLine("Press PLAY to start tracking...");
        telemetry.update();
    }


    @Override
    public void onUpdate() {
        super.onUpdate();
        odom.updateOdom();

        telemetry.addData("x (inch)", odom.getROx1());
        telemetry.addData("y (inch)", odom.getROy1());
        telemetry.addData("h (radians)", odom.getROh());
        telemetry.addData("distance", odom.getDistance());
        telemetry.update();


        telemetry.addData("LY", Gamepads.gamepad1().leftStickY());
        telemetry.addData("LX", Gamepads.gamepad1().leftStickX());
        telemetry.addData("RX", Gamepads.gamepad1().rightStickX());

        sleep(50); // ~20Hz loop

    }
}
