package org.firstinspires.ftc.teamcode.subsystems.lower

import com.acmerobotics.dashboard.config.Config
import android.graphics.Color
import com.qualcomm.robotcore.hardware.ColorSensor

enum class BallColor {
  NONE,
  GREEN,
  PURPLE
}
@Config
class MagazineColorSensors(val sensorFront: ColorSensor, val sensorBack: ColorSensor) {
  //tune
  private val GreenHue = arrayOf(70, 170)
  private val GreenSat = 40
  private val GreenVal = 35
  private val PurpleHue = arrayOf(250, 300)
  private val PurpleSat =40
  private val PurpleVal=50

  fun BallOrder() : Array<BallColor>{
    val front32 = sensorFront.argb()
    val back32  = sensorBack.argb()
    val detections =arrayOf(Detect(front32), Detect(back32))
    return detections
  }

  private fun Detect( color: Int): BallColor{
    val hsv = FloatArray(3)
    Color.colorToHSV(color,hsv)
    if(GreenHue[0]<hsv[0] && GreenHue[1]>hsv[0] && GreenSat < hsv[1] && GreenVal < hsv[2]) {
      return BallColor.GREEN
    }
    if(PurpleHue[0]<hsv[0] && PurpleHue[1]>hsv[0] && PurpleSat < hsv[1] && PurpleVal < hsv[2]) {
      return BallColor.PURPLE
    }
    return BallColor.NONE
    //// I hate not having pointers
    //val front = IntArray(4){ i->(color.toLong() shr (i*8)).toByte().toInt()}
  }

}
