package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.Details;
import org.firstinspires.ftc.teamcode.util.TuningController;

import static org.firstinspires.ftc.teamcode.util.Details.packet;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Deposit Test", group="TeleOp")
public class sampleDeposit extends LinearOpMode
{
    // Declare OpMode members.
    Deposit deposit;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        Details.telemetry = telemetry;
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap, intake);
        TuningController<Deposit.State> tuningController = new TuningController<>(Deposit.State.values(), 2);
        waitForStart();
        tuningController.start();
        while(opModeIsActive()) {
            intake.setPower(gamepad1.right_trigger);
            packet = new TelemetryPacket();
             deposit.update();
             intake.update();
             deposit.setState(tuningController.update());
             if (packet != null) dashboard.sendTelemetryPacket(packet);
             telemetry.addData("Target Height: ", deposit.getState().dist);
             telemetry.addData("Actual Height: ", Deposit.ticksToInches(deposit.slides.getCurrentPosition()));
         }

    }


}