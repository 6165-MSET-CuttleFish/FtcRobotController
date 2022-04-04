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
import org.firstinspires.ftc.teamcode.roadrunnerext.toMeters
import org.firstinspires.ftc.teamcode.util.field.Balance
import org.firstinspires.ftc.teamcode.util.field.Context
import org.firstinspires.ftc.teamcode.util.field.Context.balance

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
    lateinit var intakeButton: TriggerReader
    lateinit var ninjaMode: TriggerReader
    lateinit var liftButton: TriggerReader
    lateinit var softDump: TriggerReader
    lateinit var levelIncrement: ButtonReader
    lateinit var levelDecrement: ButtonReader
    lateinit var dumpButton: ButtonReader
    lateinit var tippedToward: ButtonReader
    lateinit var tippedAway: ButtonReader
    lateinit var capHorizontalInc: ButtonReader
    lateinit var capVerticalInc: ButtonReader
    lateinit var capHorizontalDec: ButtonReader
    lateinit var capVerticalDec: ButtonReader
    lateinit var capRetract: ButtonReader
    lateinit var intakeCounterBalance: ButtonReader
    lateinit var depositLift: ToggleButtonReader
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
            TriggerReader(secondary, GamepadKeys.Trigger.RIGHT_TRIGGER).also {
                intakeButton = it
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
            ButtonReader(secondary, GamepadKeys.Button.LEFT_BUMPER).also {
                tippedAway = it
            },
            ButtonReader(secondary, GamepadKeys.Button.RIGHT_BUMPER).also {
                tippedToward = it
            },
            ToggleButtonReader(primary, GamepadKeys.Button.LEFT_BUMPER).also {
                depositLift = it
            },
            ButtonReader(primary, GamepadKeys.Button.RIGHT_BUMPER).also {
                dumpButton = it
            },
            ButtonReader(primary, GamepadKeys.Button.X).also {
                capRetract = it
            },
            ButtonReader(secondary, GamepadKeys.Button.A).also {
                intakeCounterBalance = it
            },
            TriggerReader(primary, GamepadKeys.Trigger.RIGHT_TRIGGER).also {
                softDump = it
            }
        )
        Deposit.allowLift = false
        waitForStart()
        var liftIndication = false
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
                liftIndication = true
            }
            if (liftIndication) {
                Deposit.allowLift = !Deposit.allowLift
                liftIndication = false
            }
            if (intakeCounterBalance.wasJustPressed()) {
                Deposit.shouldCounterBalance = !Deposit.shouldCounterBalance
            }
            if (ninjaMode.isDown) drivePower = drivePower.times(0.60)
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
            } else {
                capstone.setTape(-Math.abs(gamepad2.left_stick_y).toDouble())
            }
            robot.setWeightedDrivePower(drivePower)
            setIntake()
            setDeposit()
            setCarousel()
            if (tippedAway.isDown && tippedToward.isDown) {
                balance = Balance.BALANCED
            } else if (tippedAway.isDown) {
                balance = Balance.AWAY
            } else if (tippedToward.isDown) {
                balance = Balance.TOWARD
            }
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
            }
            deposit.setLevel(defaultDepositState)
        } else if (levelDecrement.wasJustPressed()) {
            when (defaultDepositState) {
                Deposit.Level.LEVEL3 -> defaultDepositState = Deposit.Level.LEVEL2
                Deposit.Level.LEVEL2 -> defaultDepositState = Deposit.Level.LEVEL1
            }
            deposit.setLevel(defaultDepositState)
        }
        if (dumpButton.wasJustPressed()) {
            if (Deposit.isLoaded) {
                deposit.dump()
            } else {
                Deposit.isLoaded = true
            }
        }
        if (gamepad1.right_trigger > 0.5) {
            deposit.softDump()
        }
    }

    fun setCarousel() {
        carousel.setPower(gamepad2.right_stick_y.toDouble())
    }
}