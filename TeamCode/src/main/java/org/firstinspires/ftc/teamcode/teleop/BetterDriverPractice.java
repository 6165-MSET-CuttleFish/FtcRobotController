package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.util.opmode.ImprovedOpMode;

@TeleOp
@Disabled
public class BetterDriverPractice extends ImprovedOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            update();
            Pose2d drivePower = new Pose2d(
                    -gamepad1.left_stick_y,
                    0,
                    -gamepad1.right_stick_x
            );
            if (ninjaMode.isDown()) drivePower = drivePower.div(2);
            robot.setWeightedDrivePower(drivePower);
            setIntake();
            setDeposit();
            setCarousel();
        }
    }

    void setIntake() {
        if (intakeButton.isDown()) {
            intake.setPower(1);
        } else if (outtakeButton.isDown()) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
    }
    Deposit.State defaultDepositState = Deposit.State.LEVEL3;

    void setDeposit() {
        if (levelIncrementor.wasJustPressed()) {
            switch (defaultDepositState) {
                case LEVEL2:
                    defaultDepositState = Deposit.State.LEVEL3;
                    break;
                case IDLE:
                    defaultDepositState = Deposit.State.LEVEL2;
                    break;
            }
        } else if (levelDecrementor.wasJustPressed()) {
            switch (defaultDepositState) {
                case LEVEL3:
                    defaultDepositState = Deposit.State.LEVEL2;
                    break;
                case LEVEL2:
                    defaultDepositState = Deposit.State.IDLE;
                    break;
            }
        }

        deposit.setState(defaultDepositState);

        if (dumpButton.wasJustPressed()) {
            deposit.platform.dump();
        }
    }

    void setCarousel() {
        if(carouselButton.getState()) {
            // carousel.setState(Carousel.State.ON);
        } else {
            // carousel.setState(Carousel.State.IDLE);
        }
    }
}
