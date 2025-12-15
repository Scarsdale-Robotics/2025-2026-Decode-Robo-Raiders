package org.firstinspires.ftc.teamcode.subsystems.localization

import android.graphics.Color
import com.qualcomm.robotcore.hardware.ColorSensor

enum class BallColor{
  NONE,
  GREEN,
  PURPLE
}
class MagazineColorSensors(val sensorFront: ColorSensor, val sensorBack: ColorSensor) {
  private double[2] GreenHue;
  private double GreenSat;
  private double GreenVal;
  private double[2] PurpleHue;
  private double PurpleSat;
  private double PurpleVal;

  fun BallOrder() : Array<BallColor>{
    val front32 = sensorFront.argb()
  }

  private fun Detect( color: Int): BallColor{
    val hsv = FloatArray(3)
    Color.colorToHSV(color,hsv)
    if(GreenHue[0]<hsv[0] && GreenHue[1]>hsv[0] && GreenSat < hsv[1] && GreenVal < hsv[2]) {
      return GREEN;
    }
    if(PurpleHue[0]<hsv[0] && PurpleHue[1]>hsv[0] && PurpleSat < hsv[1] && PurpleVal < hsv[2]) {
      return PURPLE;
    }
    return NONE
    //// I hate not having pointers
    //val front = IntArray(4){ i->(color.toLong() shr (i*8)).toByte().toInt()}
  }

}
