package org.firstinspires.ftc.teamcode.opmodes.teleop.LocalTests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "motorTest")
@Configurable
public class BasicOpMode extends LinearOpMode {

    DcMotorEx frontR = hardwareMap.get(DcMotorEx.class, "rfw");
    DcMotor frontL = hardwareMap.get(DcMotor.class, "lfw");
    DcMotor backR = hardwareMap.get(DcMotor.class, "rbw");
    DcMotor backL = hardwareMap.get(DcMotor.class, "lbw");
    DcMotor tp = hardwareMap.get(DcMotor.class, "turret_phi");
    DcMotor shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
    DcMotor magazine = hardwareMap.get(DcMotor.class, "magazine");
    DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor[] motors = { frontR, frontL, backR, backL, tp, shooter2, magazine, intake };

        for(int i = 0; i< motors.length; i++){
            motors[i].setPower(1);
            sleep(5000);
            motors[i].setPower(-1);
            sleep(5000);
        }

    }
}
