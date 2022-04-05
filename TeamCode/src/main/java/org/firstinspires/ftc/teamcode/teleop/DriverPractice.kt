package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.TriggerReader
import com.arcrobotics.ftclib.gamepad.ButtonReader
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader
import kotlin.Throws
import org.firstinspires.ftc.teamcode.util.field.OpModeType
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.drive.opmode.ForceCalculator
import org.firstinspires.ftc.teamcode.modules.capstone.Capstone
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.toMeters
import org.firstinspires.ftc.teamcode.util.field.Context

@TeleOp
class DriverPractice : LinearOpMode() {
    enum class Mode {
        DRIVING, ENDGAME
    }

    lateinit var intake: Intake
    lateinit var deposit: Deposit
    lateinit var carousel: Carousel
    lateinit var capstone: Capstone
    var mode = Mode.DRIVING
    var toggleMode = false
    lateinit var primary: GamepadEx
    lateinit var secondary: GamepadEx
    lateinit var ninjaMode: TriggerReader
    lateinit var liftButton: TriggerReader
    lateinit var softDump1: TriggerReader
    lateinit var hardDump1: ButtonReader
    lateinit var softDump2: TriggerReader
    lateinit var hardDump2: ButtonReader
    lateinit var levelIncrement: ButtonReader
    lateinit var levelDecrement: ButtonReader

    lateinit var capHorizontalInc: ButtonReader
    lateinit var capVerticalInc: ButtonReader
    lateinit var capHorizontalDec: ButtonReader
    lateinit var capVerticalDec: ButtonReader
    lateinit var capRetract: ButtonReader
    lateinit var depositLift: ToggleButtonReader
    lateinit var ninjaOverride: ToggleButtonReader
    var defaultDepositState = Deposit.Level.LEVEL3
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = Robot<Any?>(this, OpModeType.TELE)
        intake = robot.intake
        deposit = robot.deposit
        carousel = robot.carousel
        capstone = robot.capstone
        primary = GamepadEx(gamepad1)
        secondary = GamepadEx(gamepad2)
        val keyReaders = arrayOf(
            ButtonReader(primary, GamepadKeys.Button.DPAD_RIGHT).also {
                capHorizontalInc = it
            },
            ButtonReader(primary, GamepadKeys.Button.DPAD_LEFT).also {
                capHorizontalDec = it
            },
            ButtonReader(primary, GamepadKeys.Button.DPAD_UP).also {
                capVerticalInc = it
            },
            ButtonReader(primary, GamepadKeys.Button.DPAD_DOWN).also {
                capVerticalDec = it
            },
            TriggerReader(primary, GamepadKeys.Trigger.LEFT_TRIGGER).also {
                ninjaMode = it
            },
            ButtonReader(secondary, GamepadKeys.Button.DPAD_UP).also {
                levelIncrement = it
            },
            ButtonReader(secondary, GamepadKeys.Button.DPAD_DOWN).also {
                levelDecrement = it
            },
            TriggerReader(secondary, GamepadKeys.Trigger.LEFT_TRIGGER).also {
                liftButton = it
            },
            ToggleButtonReader(primary, GamepadKeys.Button.LEFT_BUMPER).also {
                depositLift = it
            },
            ButtonReader(primary, GamepadKeys.Button.RIGHT_BUMPER).also {
                hardDump1 = it
            },
            ButtonReader(secondary, GamepadKeys.Button.RIGHT_BUMPER).also {
                hardDump2 = it
            },
            ButtonReader(primary, GamepadKeys.Button.X).also {
                capRetract = it
            },
            TriggerReader(primary, GamepadKeys.Trigger.RIGHT_TRIGGER).also {
                softDump1 = it
            },
            TriggerReader(secondary, GamepadKeys.Trigger.RIGHT_TRIGGER).also {
                softDump2 = it
            },
            ToggleButtonReader(primary, GamepadKeys.Button.LEFT_STICK_BUTTON).also {
                ninjaOverride = it
            }
        )
        deposit.liftDown()
        waitForStart()
        while (opModeIsActive()) {
            robot.update()
            for (reader in keyReaders) {
                reader.readValue()
            }
            var drivePower = Pose2d(
                (-gamepad1.left_stick_y).toDouble(),
                0.0,
                (-gamepad1.right_stick_x).toDouble()
            )
            if (intake.containsBlock && intake.state === Intake.State.OUT) {
                gamepad1.rumble(500)
                gamepad2.rumble(500)
            }
            if (depositLift.wasJustPressed()) {
                deposit.toggleLift()
            }
            if (ninjaMode.isDown || (deposit.platformIsOut() && !ninjaOverride.state)) drivePower *= 0.6
            if (gamepad1.touchpad) {
                if (!toggleMode) {
                    mode = if (mode == Mode.DRIVING) {
                        capstone.setTape(0.0)
                        capstone.active()
                        Mode.ENDGAME
                    } else {
                        capstone.retract()
                        Mode.DRIVING
                    }
                }
                toggleMode = true
            } else {
                toggleMode = false
            }
            if (capRetract.wasJustPressed()) {
                if (capstone.state == Capstone.State.ACTIVE) {
                    capstone.retract()
                } else {
                    capstone.idle()
                }
            }
            if (mode == Mode.ENDGAME) {
                drivePower = Pose2d(
                    (-gamepad2.left_stick_y).toDouble(),
                    0.0,
                    0.0
                ).div(8.0)
                setCapstone()
            }
            robot.setWeightedDrivePower(drivePower)
            if (deposit.state == Deposit.State.IN) setIntake()
            setDeposit()
            setCarousel()
            val accel = robot.getPoseAcceleration() ?: Pose2d()
            Context.packet.put("Linear Acceleration", accel.toMeters().x)
            Context.packet.put("Angular Acceleration", accel.heading)
            Context.packet.put("Force Estimate", accel.toMeters().x * ForceCalculator.mass)
        }
    }

    fun setCapstone() {
        capstone.setTape((gamepad1.right_trigger - gamepad1.left_trigger).toDouble())
        capstone.setVerticalTurret(gamepad1.left_stick_y.toDouble())
        capstone.setHorizontalTurret(gamepad1.right_stick_x.toDouble())
        carousel.setPower(gamepad2.right_stick_y.toDouble())
        if (capHorizontalInc.wasJustPressed()) {
            capstone.incrementHorizontal(1.0)
        } else if (capHorizontalDec.wasJustPressed()) {
            capstone.incrementHorizontal(-1.0)
        }
        if (capVerticalInc.wasJustPressed()) {
            capstone.incrementVertical(1.0)
        } else if (capVerticalDec.wasJustPressed()) {
            capstone.incrementVertical(-1.0)
        }
    }

    fun setIntake() {
        intake.setPower((gamepad2.right_trigger + gamepad2.left_trigger).toDouble())
    }

    fun setDeposit() {
        if (levelIncrement.wasJustPressed()) {
            when (defaultDepositState) {
                Deposit.Level.LEVEL2 -> defaultDepositState = Deposit.Level.LEVEL3
                Deposit.Level.LEVEL1 -> defaultDepositState = Deposit.Level.LEVEL2
                else -> {}
            }
            deposit.setLevel(defaultDepositState)
        } else if (levelDecrement.wasJustPressed()) {
            when (defaultDepositState) {
                Deposit.Level.LEVEL3 -> defaultDepositState = Deposit.Level.LEVEL2
                Deposit.Level.LEVEL2 -> defaultDepositState = Deposit.Level.LEVEL1
                else -> {}
            }
            deposit.setLevel(defaultDepositState)
        }
        if (hardDump1.wasJustPressed()) {
            if (Deposit.isLoaded) {
                deposit.dump()
                gamepad1.rumble(200)
                gamepad2.rumble(200)
            } else if (deposit.state == Deposit.State.IN && !deposit.isTransitioningState()){
                Deposit.isLoaded = true
            }
        }
        if (softDump1.isDown) {
            deposit.softDump()
            gamepad1.rumble(100)
            gamepad2.rumble(100)
        }
    }

    fun setCarousel() {
        carousel.setPower(gamepad2.right_stick_y.toDouble())
    }
}