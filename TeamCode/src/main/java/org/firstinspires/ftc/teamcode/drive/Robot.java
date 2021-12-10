package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;

import org.firstinspires.ftc.teamcode.localizers.t265.Easy265;
import org.firstinspires.ftc.teamcode.localizers.t265.T265Localizer;
import org.firstinspires.ftc.teamcode.util.roadrunnerext.ImprovedTankDrive;

import org.firstinspires.ftc.teamcode.util.roadrunnerext.ImprovedTrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.capstone.Capstone;
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.modules.vision.Detector;
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
import org.firstinspires.ftc.teamcode.util.roadrunnerext.ImprovedRamsete;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.Arrays;
import java.util.List;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.COOLDOWN_TIME;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_CURRENT;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_CURRENT_OVERFLOW_TIME;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.admissibleDistance;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.admissibleError;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.admissibleHeading;
import static org.firstinspires.ftc.teamcode.util.field.Details.location;
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
import static org.firstinspires.ftc.teamcode.util.field.Details.telemetry;

/**
 * This class represents the robot and its drivetrain
 * @author Ayush Raman
 */
@Config
public class Robot extends ImprovedTankDrive {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam
    private OpenCvCamera webcam;
    private final Detector detector = new Detector();
    private final double pitchOffset;
    public static double div = 1;

    final HardwareMap hardwareMap;

    private final Module[] modules;
    public Intake intake;
    public Deposit deposit;
    public Carousel carousel;
    public Capstone capstone;

    private final BNO055IMU imu;
    private final List<DcMotorEx> motors, leftMotors, rightMotors;
    private final VoltageSensor batteryVoltageSensor;

    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(6,0,0.001);
    public static PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0,3,0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8,0,0);

    public static double VX_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 2;
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);
    private final TrajectorySequenceRunner trajectorySequenceRunner;
    private final FtcDashboard dashboard;

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
        super(TRACK_WIDTH, opMode.hardwareMap.voltageSensor.iterator().next());
        dashboard = FtcDashboard.getInstance();
        Details.opModeType = type;
        Details.alliance = alliance;
        robotPose = pose2d;
        if (opModeType == OpModeType.AUTO) robotPose = FrequentPositions.startingPosition();
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
        dashboard.setTelemetryTransmissionInterval(25);
        ImprovedTrajectoryFollower follower = new ImprovedRamsete();
        // follower = new TankPIDVAFollower(AXIAL_PID, CROSS_TRACK_PID, new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
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
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "br"), // enc
                rightFront = hardwareMap.get(DcMotorEx.class, "fr"), //
                rightMid = hardwareMap.get(DcMotorEx.class, "mr"); //
        modules = new Module[]{
                intake = new Intake(hardwareMap),
                deposit = new Deposit(hardwareMap, intake),
                carousel = new Carousel(hardwareMap),
                capstone = new Capstone(hardwareMap),
        };
        motors = Arrays.asList(leftFront, leftRear, leftMid, rightFront, rightRear, rightMid);
        leftMotors = Arrays.asList(leftFront, leftRear, leftMid);
        rightMotors = Arrays.asList(rightFront, rightRear, rightMid);
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setCurrentAlert(MAX_CURRENT, CurrentUnit.MILLIAMPS);
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
        if (opModeType != OpModeType.TELE) {
//            Easy265.initWithoutStop(opMode, this);
//            setLocalizer(new T265Localizer());
        }
        if (opModeType == OpModeType.AUTO) {
            // autoInit();
        }
        pitchOffset = -imu.getAngularOrientation().secondAngle;
        setPoseEstimate(robotPose);
        telemetry.clear();
        telemetry.addData("Init", "Complete");
        telemetry.update();
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
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode) {
            }
        });
        dashboard.startCameraStream(webcam, 30);
    }

    public void setPipeline(OpenCvPipeline pipeline) {
        webcam.setPipeline(pipeline);
    }

    public void turnOffVision() {
        webcam.closeCameraDeviceAsync(() -> webcam.stopStreaming());
        webcam.closeCameraDevice();
    }

    public void scan() {
        location = detector.getLocation();
    }

    public static Deposit.State getLevel(Detector.Location location) {
        switch (location) {
            case LEFT: return Deposit.State.IDLE;
            case MIDDLE: return Deposit.State.LEVEL2;
            case RIGHT: return Deposit.State.LEVEL3;
        }
        return Deposit.State.LEVEL3;
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
                        .build()
        );
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

    boolean isRobotDisabled;
    ElapsedTime currentTimer = new ElapsedTime();
    ElapsedTime cooldown = new ElapsedTime();
    ElapsedTime loopTime = new ElapsedTime();

    public void update() {
        updatePoseEstimate();
        if (!Thread.currentThread().isInterrupted()) {
            Details.robotPose = getPoseEstimate();
        }
        for (Module module : modules) {
            module.update();
        }
        Details.packet.put("Loop Time", loopTime.milliseconds());
        loopTime.reset();
        if (admissibleDistance != admissibleError.getX() || admissibleHeading != Math.toDegrees(admissibleError.getHeading())) {
            admissibleError = new Pose2d(admissibleDistance, admissibleDistance, Math.toRadians(admissibleHeading));
        }
        boolean anyMotorIsOverCurrent = false;
        for (DcMotorEx motor : motors) {
            if (motor.isOverCurrent()) anyMotorIsOverCurrent = true;
        }
        if (anyMotorIsOverCurrent && currentTimer.seconds() > MAX_CURRENT_OVERFLOW_TIME) {
            isRobotDisabled = true;
            cooldown.reset();
        } else {
            if (cooldown.seconds() > COOLDOWN_TIME) isRobotDisabled = false;
            if (!anyMotorIsOverCurrent) currentTimer.reset();
        }
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        waitForIdle(() -> {
        });
    }

    /**
     * @return Whether the robot's current state is potentially hazardous to operate in
     */
    public boolean isHazardous() {
        return false;
    }

    public boolean isDoingWorkHelper(Module... modules) {
        for (Module module : modules) {
            if (module.isDoingWork()) {
                return true;
            }
            if (module.nestedModules.length != 0) {
                if (isDoingWorkHelper(module.nestedModules)) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @return Whether the robot is currently doing work
     */
    public boolean isDoingWork() {
        return isDoingWorkHelper(modules);
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
        Pose2d vel = new Pose2d(
                isRobotDisabled ? 0 : drivePower.getX(),
                isRobotDisabled ? 0 : drivePower.getY(),
                isRobotDisabled ? 0 : drivePower.getHeading() * 2
        );
        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getHeading()) > 1 && !isRobotDisabled) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());
            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    0,
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }
        setDrivePower(vel.div(div));
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
        double pitch = getPitch();
        return Arrays.asList(leftSum * Math.cos(pitch), rightSum * Math.cos(pitch));
    }

    public List<Double> getWheelVelocities() {
        double leftSum = 0, rightSum = 0;
        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += encoderTicksToInches(leftMotor.getVelocity());
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += encoderTicksToInches(rightMotor.getVelocity());
        }
        double pitch = getPitch();
        return Arrays.asList(leftSum * Math.cos(pitch), rightSum * Math.cos(pitch));
    }

    @Override
    public void setMotorPowers(double v, double v1) {
        for (DcMotorEx leftMotor : leftMotors) {
            leftMotor.setPower(v * 1.03092783505);
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightMotor.setPower(v1);
        }
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

    public double getPitch() {
        return -imu.getAngularOrientation().secondAngle - pitchOffset;
    }
}