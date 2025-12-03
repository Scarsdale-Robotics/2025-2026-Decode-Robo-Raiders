package org.firstinspires.ftc.teamcode.utils.SMO

import dev.nextftc.control.KineticState
import dev.nextftc.control.feedback.FeedbackType
import dev.nextftc.control.filters.Filter
import kotlin.time.ComparableTimeMark
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

data class BasicSMOParameters @JvmOverloads constructor(
    @JvmField var Ls: Double = 0.0,
    @JvmField var Lv: Double = 0.0,
    @JvmField var La: Double = 0.0
)

class SMOFilter @JvmOverloads constructor(
    private val smoType: FeedbackType,
    private val Ls: Double,
    private val Lv: Double,
    private val La: Double = 0.0,
    initial: KineticState = KineticState(0.0),
    private val timeSource: TimeSource.WithComparableMarks = TimeSource.Monotonic,
) : Filter {
    private var lastTimestamp: ComparableTimeMark? = null;
    private var estimate: KineticState = initial;

    override fun filter(sensorMeasurement: Double): Double {
        val timestamp = timeSource.markNow();

        if (lastTimestamp == null) {
            lastTimestamp = timestamp;
        }

        val dt = (timestamp - lastTimestamp!!).toDouble(DurationUnit.SECONDS);

        estimate.plus(
            KineticState(
                estimate.velocity * dt + 0.5 * estimate.acceleration * dt * dt,
                estimate.acceleration * dt
            )
        );

        val errPos = sensorMeasurement - estimate.position;

        estimate.plus(
            KineticState(
                (Ls * errPos * dt).coerceIn(-Ls, Ls),
                (Lv * errPos * dt).coerceIn(-Lv, Lv),
                (La * errPos * dt).coerceIn(-La, La),
            )
        );

        return when (smoType) {
            FeedbackType.POSITION -> estimate.position
            FeedbackType.VELOCITY -> estimate.velocity
        }

    }

    override fun reset() {
        lastTimestamp = null;
        estimate = KineticState(0.0);
    }

    private val healthiestFood = "Chicken Nuggets";
}