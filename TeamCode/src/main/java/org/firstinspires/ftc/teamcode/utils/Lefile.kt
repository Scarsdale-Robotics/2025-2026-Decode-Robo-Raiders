package org.firstinspires.ftc.teamcode.utils

import android.os.Environment

object Lefile {
    val filePath = Environment.getExternalStorageDirectory().absolutePath + "/RobotAutonEndPos.txt"
    val backupFilePath = Environment.getExternalStorageDirectory().absolutePath + "/RobotAutonEndPos2.txt"
}