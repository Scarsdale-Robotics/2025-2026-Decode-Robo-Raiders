package org.firstinspires.ftc.teamcode.subsystems.lower

import android.graphics.Color
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.HardwareMap

enum class BallColor(val number: Int) {
  NONE(0),
  GREEN(1),
  PURPLE(2);

  fun getNum(): Int {
    return number
  }
}


@Config
class MagazineColorSensors(val hardwareMap: HardwareMap) {


  //tune
  private val greenHue = arrayOf(70, 170)
  private val greenSat = 40
  private val greenVal = 35
  private val purpleHue = arrayOf(250, 300)
  private val purpleSat = 40
  private val purpleVal = 50

  private val sensorFront: ColorSensor =
    hardwareMap.get(ColorSensor::class.java, "sensorFront")
  private val sensorBack: ColorSensor =
    hardwareMap.get(ColorSensor::class.java, "sensorBack")


  fun BallOrder() : Array<BallColor>{
    val front32 = sensorFront.argb()
    val back32  = sensorBack.argb()
    val detections = arrayOf(detect(front32), detect(back32))
    return detections
  }

  private fun detect(color: Int): BallColor{
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
