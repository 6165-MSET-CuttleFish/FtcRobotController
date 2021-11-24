package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.localizers.Easy265;
import org.firstinspires.ftc.teamcode.localizers.T265Localizer;
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment.FutureSegment;
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.Details;
import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.field.Side;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.Arrays;
import java.util.List;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.teamcode.util.field.Details.opModeType;
import static org.firstinspires.ftc.teamcode.util.field.Details.robotPose;
import static org.firstinspires.ftc.teamcode.util.field.Details.alliance;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.util.field.Details.side;

/**
 * This class represents the robot and its drivetrain
 * @author Ayush Raman
 */
@Config
public class Robot extends TankDrive {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final int HORIZON = 100; // horizon value to tune
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam
    public OpenCvCamera webcam;

    final OpMode opMode;
    final HardwareMap hardwareMap;
    Telemetry telemetry;

    private final Module[] modules;
    public Intake intake;
    public Deposit deposit;
    public Carousel carousel;

    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double VX_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 5;
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);
    private final TrajectorySequenceRunner trajectorySequenceRunner;
    private final List<DcMotorEx> motors, leftMotors, rightMotors;
    private final VoltageSensor batteryVoltageSensor;
    FtcDashboard dashboard;
    private BNO055IMU imu;


    public static Pose2d flipSide(Pose2d pose2d) {
        return new Pose2d(pose2d.getX(), -pose2d.getY(), -pose2d.getHeading());
    }

    public static Pose2d midPoint() {
        Pose2d regular = new Pose2d(-55.0, -55.0, Math.toRadians(-210.0));
        return alliance == Alliance.RED ? regular : flipSide(regular);
    }

    public static Pose2d startingPosition() {
        Pose2d regular = new Pose2d(-36, -58, Math.toRadians(-90));
        return alliance == Alliance.RED ? regular : flipSide(regular);
    }

    public static Pose2d cycleDump() {
        Pose2d regular = new Pose2d(-28.0, -31.0, Math.toRadians(20.0));
        return alliance == Alliance.RED ? regular : flipSide(regular);
    }

    public static Pose2d[] duckLocations() {
        Pose2d[] values;
        if (side == Side.CAROUSEL) {
            values = new Pose2d[] {
                    new Pose2d(-32.0, -50.0, Math.toRadians(0)),
                    new Pose2d(-40.0, -50.0, Math.toRadians(0)),
                    new Pose2d(-50.0, -50.0, Math.toRadians(0))
            };
        } else {
            values = new Pose2d[] {
                    new Pose2d(3.0, -50.0),
                    new Pose2d(11.5, -50.0),
                    new Pose2d(20.0, -50.0)
            };
        }
        if (alliance == Alliance.BLUE) {
            for (int i = 0; i < values.length; i++) {
                values[i] = flipSide(values[i]);
            }
        }
        return values;
    }

    public Robot(OpMode opMode, Pose2d pose2d) {
        this(opMode, pose2d, OpModeType.NONE, Alliance.NONE);
    }

    public Robot(OpMode opMode, OpModeType type, Alliance alliance) {
        this(opMode, Details.robotPose, type, alliance);
    }

    public Robot(OpMode opMode, OpModeType type) {
        this(opMode, Details.robotPose, type, alliance);
    }

    public Robot(OpMode opMode) {
        this(opMode, new Pose2d(0, 0, 0), OpModeType.NONE, Alliance.NONE);
    }

    public Robot(OpMode opMode, Pose2d pose2d, OpModeType type, Alliance alliance) {
        super(kV, kA, kStatic, TRACK_WIDTH);
        Details.opModeType = type;
        Details.robotPose = pose2d;
        Details.alliance = alliance;
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Details.telemetry = telemetry;
        TrajectoryFollower follower = new TankPIDVAFollower(AXIAL_PID, CROSS_TRACK_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "fl"), //
                leftRear = hardwareMap.get(DcMotorEx.class, "bl"), //
                leftMid = hardwareMap.get(DcMotorEx.class, "ml"); // enc
        DcMotorEx  rightRear = hardwareMap.get(DcMotorEx.class, "br"), // enc
                rightFront = hardwareMap.get(DcMotorEx.class, "fr"), //
                rightMid = hardwareMap.get(DcMotorEx.class, "mr"); //
        modules = new Module[]{
                intake = new Intake(hardwareMap),
                deposit = new Deposit(hardwareMap, intake),
                //carousel = new Carousel(hardwareMap),
        };
        motors = Arrays.asList(leftFront, leftRear, leftMid, rightFront, rightRear, rightMid);
        leftMotors = Arrays.asList(leftFront, leftRear, leftMid);
        rightMotors = Arrays.asList(rightFront, rightRear, rightMid);
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }
        for (DcMotorEx motor : rightMotors) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
        if (opModeType == OpModeType.AUTO) {
            autoInit();
            Easy265.init(opMode, leftMid, rightMid, imu);
            setLocalizer(new T265Localizer());
        }
        setPoseEstimate(robotPose);
    }

    public void autoInit() {
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

//        RingLocalizer.CAMERA_WIDTH = CAMERA_WIDTH;
//
//        RingLocalizer.HORIZON = HORIZON;
        // webcam.setPipeline(pipeline = new UGContourRingPipeline());
        webcam.openCameraDevice();
        webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
        //dashboard.startCameraStream(webcam, 30);
    }

    public void switchPipeline(OpenCvPipeline pipeline) {
        webcam.setPipeline(pipeline);
    }

    public void turnOffVision() {
        // webcam.closeCameraDeviceAsync(() -> webcam.stopStreaming());
        webcam.closeCameraDevice();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(Details.robotPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public TrajectorySequenceBuilder futureBuilder(FutureSegment futureSegment) {
        return new TrajectorySequenceBuilder(
                futureSegment.getStartPose(),
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void turn(double angle, Runnable runnable) {
        turnAsync(angle);
        waitForIdle(runnable);
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build());
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public boolean intakeReq = false;

    double current;

    public void update() {
        updatePoseEstimate();
        if (!Thread.currentThread().isInterrupted()) {
            Details.robotPose = getPoseEstimate();
        }
        for (Module module : modules) {
            module.update();
        }
        for (DcMotorEx motor : motors) {
            telemetry.addData(motor.getDeviceName(), motor.getPower());
            current += motor.getCurrent(CurrentUnit.MILLIAMPS);
        }
        if (current > 30000) {
            setMotorPowers(0, 0);
        }
        Details.packet.put("Total Motor Current", current);
        telemetry.addData("Motor Current", current);
        Log.println(Log.INFO, "motor_current", ""+current);
        current = 0;
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        waitForIdle(() -> {
        });
    }

    /**
     *
     * @return Whether the robot's current state is potentially hazardous to operate in
     */
    public boolean isHazardous() {
        return false;
    }

    /**
     * Hold the robot in place until all hazardous actions are completed
     */
    public void waitForActionsCompleted() {
        update();
        while (isHazardous() && !Thread.currentThread().isInterrupted()) {
            update();
        }
    }

    private void waitForIdle(Runnable block) {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            block.run();
            update();
        }
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        if (current > 3000) return;
        Pose2d vel = new Pose2d(
                drivePower.getX(),
                drivePower.getY(),
                drivePower.getHeading() * 2
        );
        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    0,
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
            telemetry.addData("denom", denom);
            telemetry.addData("VX", VX_WEIGHT * Math.abs(drivePower.getX()));
        }
        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        double leftSum = 0, rightSum = 0;
        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += encoderTicksToInches(leftMotor.getCurrentPosition());
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += encoderTicksToInches(rightMotor.getCurrentPosition());
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
    }

    public List<Double> getWheelVelocities() {
        double leftSum = 0, rightSum = 0;
        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += encoderTicksToInches(leftMotor.getVelocity());
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += encoderTicksToInches(rightMotor.getVelocity());
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
    }

    @Override
    public void setMotorPowers(double v, double v1) {
        for (DcMotorEx leftMotor : leftMotors) {
            leftMotor.setPower(v);
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightMotor.setPower(v1);
        }
    }

    public void correctHeading() {
        Pose2d currPose = getPoseEstimate();
        setPoseEstimate(new Pose2d(currPose.getX(), currPose.getY(), getExternalHeading()));
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new TankVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}