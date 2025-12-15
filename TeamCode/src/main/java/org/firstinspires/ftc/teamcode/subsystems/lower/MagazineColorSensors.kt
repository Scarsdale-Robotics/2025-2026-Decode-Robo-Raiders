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
  private val greenHue = arrayOf(70, 170)
  private val greenSat = 40
  private val greenVal = 35
  private val purpleHue = arrayOf(250, 300)
  private val purpleSat =40
  private val purpleVal=50

  fun BallOrder() : Array<BallColor>{
    val front32 = sensorFront.argb()
    val back32  = sensorBack.argb()
    val detections =arrayOf(detect(front32), detect(back32))
    return detections
  }

  private fun detect( color: Int): BallColor{
    val hsv = FloatArray(3)
    Color.colorToHSV(color,hsv)
    if(greenHue[0]<hsv[0] && greenHue[1]>hsv[0] && greenSat < hsv[1] && greenVal < hsv[2]) {
      return BallColor.GREEN
    }
    if(purpleHue[0]<hsv[0] && purpleHue[1]>hsv[0] && purpleSat < hsv[1] && purpleVal < hsv[2]) {
      return BallColor.PURPLE
    }
    return BallColor.NONE
  }

}
