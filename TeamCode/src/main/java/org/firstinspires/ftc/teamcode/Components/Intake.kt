package org.firstinspires.ftc.teamcode.Components

import com.qualcomm.robotcore.hardware.*

class Intake(hardwareMap: HardwareMap) : Component {
    enum class IntakeState {
        SAFE, CUSTOM_VALUE, IDLE
    }

    var intakeState = IntakeState.CUSTOM_VALUE
    var intakeMotor: DcMotor
    var intakeL: CRServo
    var intakeR: CRServo
    var drop: Servo
    override fun update() {}
    private fun setVectors(power: Double) {
        intakeL.power = power
        intakeR.power = power
    }

    //use these two functions so that it drops intake and then raises the dropper
    fun dropIntake() {
        drop.position = 0.6
    }

    fun raiseIntake() {
        drop.position = 1.0
    }

    //public double getDropPosition(){return drop.getPosition();}
    fun setPower(power: Double) {
        intakeState = IntakeState.CUSTOM_VALUE
        intakeMotor.power = power
        setVectors(power)
    }

    init {
        intakeMotor = hardwareMap.get(DcMotor::class.java, "intake")
        intakeMotor.direction = DcMotorSimple.Direction.REVERSE
        intakeMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intakeL = hardwareMap.crservo["intakeL"]
        intakeR = hardwareMap.crservo["intakeR"]
        drop = hardwareMap.servo["drop"]
        intakeR.direction = DcMotorSimple.Direction.REVERSE
    }
}