package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import dev.nextftc.ftc.NextFTCOpMode;

//Auton Naming Convention
//total slots = 4: __ __ __ __
//First slot = Name Type: Auton
//2nd slot = Classifier type (There can be multiple types on the same auto):
//1. Leaving it blank
//2. Wait (only for shooter type autons)
//3. Far (only for shooter and backup type autons)
//4. Close (only for shooter and backup type autons)
//3rd slot = Side type: Blue, Red
//4th slot = Auton type: Motif, Backup, Shooter
//Example Auton = AutonBlueCloseBackup, AutonRedWaitFarShooter ...
//Main Autons should be: Auton__WaitFarShooter & Auton__Motif

@Autonomous(name = "Auton Blue Far Backup", group = "Auton")
public class AutonBlueFarBackup extends NextFTCOpMode {
}
