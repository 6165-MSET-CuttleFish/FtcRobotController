package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader
import kotlin.Throws
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.util.field.Context
import java.util.*

@TeleOp
@Config
class InertiaTuner : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = Robot<Any?>(this)
        val gamepadEx = GamepadEx(gamepad1)
        val modeToggle = ToggleButtonReader(gamepadEx, GamepadKeys.Button.A)
        waitForStart()
        while (opModeIsActive()) {
            robot.setDriveSignal(DriveSignal( Pose2d(0.0, 0.0, Math.toRadians(50.0))))
            robot.update()
            if (modeToggle.state) {
                robot.deposit.liftUp()
                Deposit.isLoaded = true
                robot.deposit.setLevel(Deposit.Level.LEVEL3)
            } else {
                robot.deposit.liftDown()
                Deposit.isLoaded = false
            }
        }
    }
}