package org.firstinspires.ftc.teamcode.localizers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

import androidx.annotation.NonNull;

@Config
public class PerpTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.38/2;// in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 15.76; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -7.12;//0.26; // in; offset of the lateral wheel

    public static double X_LEFT_MULTIPLIER = 1.0000419; // Multiplier in the X direction
    public static double X_RIGHT_MULTIPLIER = 1.0000419; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.99404994; // Multiplier in the Y direction

    public Encoder leftEncoder, rightEncoder, frontEncoder;

    public PerpTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "fl"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "fr"));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_LEFT_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_RIGHT_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_LEFT_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_RIGHT_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}