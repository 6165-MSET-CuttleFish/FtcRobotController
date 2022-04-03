package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader
import kotlin.Throws
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.util.field.Context
import java.util.*

@TeleOp
@Config
class ForceCalculator : LinearOpMode() {
    companion object {
        @JvmField
        var power = 0.5
        @JvmField
        var mass = 19.0509 // in kg
    }
    // F = ma
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = Robot<Any?>(this)
        val gamepadEx = GamepadEx(gamepad1)
        val modeToggle = ToggleButtonReader(gamepadEx, GamepadKeys.Button.A)
        waitForStart()
        while (opModeIsActive()) {
            robot.update()
            modeToggle.readValue()
            if (modeToggle.state) {
                robot.setWeightedDrivePower(Pose2d(-power))
            } else {
                robot.setWeightedDrivePower(
                    Pose2d(
                        (-gamepad1.left_stick_y).toDouble(),
                        0.0,
                        (-gamepad1.right_stick_x).toDouble()
                    )
                )
            }
            val accel = robot.getPoseAcceleration() ?: Pose2d()
            Context.packet.put("Linear Acceleration", accel.x)
            Context.packet.put("Angular Acceleration", accel.heading)
            Context.packet.put("Force Estimate", accel.x * mass)
        }
    }
}