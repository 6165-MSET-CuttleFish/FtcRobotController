package org.firstinspires.ftc.teamcode.modules.wrappers.actuators

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.util.ElapsedTime

class ControllableCRServo(private val crServo: CRServo) {
    private var internalPosEstimate = 0.0
    private var timer = ElapsedTime()
    var isRetracting = false
    var power: Double
        get() = crServo.power
        set(val1) {
            crServo.power = val1
        }
    fun update() {
//        internalPosEstimate += power * timer.seconds()
//        timer.reset()
//        if (isRetracting) {
//            if (internalPosEstimate <= 0) {
//                power = 0.0
//                isRetracting = false
//            } else {
//                power = -1.0
//            }
//        }
    }
}