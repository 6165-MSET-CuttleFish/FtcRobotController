package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Components.localizer.T265;
import org.firstinspires.ftc.teamcode.Components.localizer.t265Localizer;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.teamcode.Components.Details.opModeType;
import static org.firstinspires.ftc.teamcode.Components.Details.side;
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
@Config
public class Robot extends MecanumDrive implements Component {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final int HORIZON = 100; // horizon value to tune
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam
    public OpenCvCamera webcam;
    private UGContourRingPipeline pipeline;
    private RingLocalizer bouncebacks;

    public UGContourRingPipeline.Height height = UGContourRingPipeline.Height.ZERO;

    public final LinearOpMode linearOpMode;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public static Vector2d goal = new Vector2d(70.5275, -32.9725);
    public static Vector2d[] powerShotLocals = {
            new Vector2d(-5.8, -6.3),
            new Vector2d(-5.8, -16),
            new Vector2d(-5.8, -22)
    };
    public static Vector2d[] powerShots = {
            new Vector2d(70.4725, -1.4725),
            new Vector2d(70.4725, -10.4725),
            new Vector2d(70.4725, -19.4725)
    };
    public static Vector2d[] dropZones = {
            new Vector2d(-5.4725, -55.4),
            new Vector2d(23, -35.4725),
            new Vector2d(45.5275, -57)
    };
    public static Vector2d rightWobble = new Vector2d(-32, -51);

    private final DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public Intake intake;
    public Shooter shooter;
    public WobbleArm wobbleArm;
    private final HashSet<Component> components;
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(25, 1.3, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(10, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);
    private final TrajectorySequenceRunner trajectorySequenceRunner;
    private final List<DcMotorEx> motors;
    private final BNO055IMU imu;
    private final VoltageSensor batteryVoltageSensor;
    public LinkedList<Runnable> actionQueue = new LinkedList<Runnable>();
    FtcDashboard dashboard;

    public Robot(LinearOpMode opMode, Pose2d pose2d) {
        this(opMode, pose2d, OpModeType.NONE);
    }

    public Robot(LinearOpMode opMode, OpModeType type) {
        this(opMode, Details.robotPose, type);
    }

    public Robot(LinearOpMode opMode) {
        this(opMode, new Pose2d(0, 0, 0), OpModeType.NONE);
    }

    public Robot(LinearOpMode opMode, Pose2d pose2d, OpModeType type) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        Details.opModeType = type;
        Details.robotPose = pose2d;
        linearOpMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        TrajectoryFollower follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
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
        leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        rightRear = hardwareMap.get(DcMotorEx.class, "br");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");
        wobbleArm = new WobbleArm(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        components = new HashSet<>(Arrays.asList(intake, shooter, wobbleArm));
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        switch(opModeType) {
            case TELE:
                setLocalizer(new t265Localizer(hardwareMap));
                break;
            default:
                setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
                break;
        }
        setPoseEstimate(Details.robotPose);
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public void setVectors() {
        switch (side) {
            case RED:
                break;
            case BLUE:
                break;
        }
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
        webcam.setPipeline(pipeline = new UGContourRingPipeline(linearOpMode.telemetry, false));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        RingLocalizer.CAMERA_WIDTH = CAMERA_WIDTH;

        RingLocalizer.HORIZON = HORIZON;

        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));
        dashboard.startCameraStream(webcam, 30);
    }

    public Trajectory ringPickup;

    public void createTraj(double endTangent) {
        TrajectoryBuilder builder;
        Vector2d[] wayPoints;
        switch (side) {
            case RED:
                builder = trajectoryBuilder(Coordinate.toPose(powerShotLocals[2], 0))
                        .splineTo(new Vector2d(15, 7), new Vector2d(10, 8).angleBetween(new Vector2d(42, 9)))
                        .splineTo(new Vector2d(42, 7), 0);
                wayPoints = bouncebacks.getVectors(getPoseEstimate()).toArray(new Vector2d[0]).clone();
                bubbleSort(wayPoints, Details.robotPose.vec());
                for (int i = 0; i < wayPoints.length; i++) {
                    builder = builder.splineTo(wayPoints[i], i < wayPoints.length - 2 ? wayPoints[i].angleBetween(wayPoints[i + 1]) : endTangent);
                }
                ringPickup = builder.splineTo(new Vector2d(55.7, -38), Math.toRadians(-75))
                        .build();
                break;
            case BLUE:
                builder = trajectoryBuilder(Coordinate.toPose(powerShotLocals[2], 0))
                        .splineTo(new Vector2d(15, 7), new Vector2d(10, 8).angleBetween(new Vector2d(42, 9)))
                        .splineTo(new Vector2d(42, 7), 0);
                wayPoints = bouncebacks.getVectors(getPoseEstimate()).toArray(new Vector2d[0]).clone();
                bubbleSort(wayPoints, new Vector2d());
                for (int i = 0; i < wayPoints.length; i++) {
                    builder = builder.splineTo(wayPoints[i], i < wayPoints.length - 2 ? wayPoints[i].angleBetween(wayPoints[i + 1]) : endTangent);
                }
                ringPickup = builder.splineTo(new Vector2d(55.7, -38), Math.toRadians(-75))
                        .build();
                break;
        }

    }

    void bubbleSort(Vector2d[] arr, Vector2d referencePose) {
        int n = arr.length;
        for (int i = 0; i < n - 1; i++)
            for (int j = 0; j < n - i - 1; j++)
                if (arr[j].distTo(referencePose) > arr[j + 1].distTo(referencePose)) {
                    // swap arr[j+1] and arr[j]
                    Coordinate temp = Coordinate.toPoint(arr[j]);
                    arr[j] = arr[j + 1];
                    arr[j + 1] = temp.toVector();
                }
    }

    public void switchPipeline(OpenCvPipeline pipeline) {
        webcam.setPipeline(pipeline);
    }

    public void scan() {
        height = pipeline.getHeight();
    }

    public void turnOffVision() {
        webcam.closeCameraDeviceAsync(() -> webcam.stopStreaming());
        dashboard.stopCameraStream();
    }

    public void release() {
        wobbleArm.claw.release();
    }

    public void intake(double intakeSpeed) {
        intake.setPower(-intakeSpeed);
    }

    public void optimalShoot() {
        shooter.tripleShot();
    }

    public void shieldUp() {
        intake.shieldUp();
    }

    public void shieldDown() {
        intake.shieldDown();
    }

    public Vector2d getDropZone() {
        if (height == UGContourRingPipeline.Height.FOUR) {
            return dropZones[2];
        } else if (height == UGContourRingPipeline.Height.ONE) {
            return dropZones[1];
        } else {
            return dropZones[0];
        }
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

    public void update() {
        updatePoseEstimate();
        if (!Thread.currentThread().isInterrupted()) {
            Details.robotPose = getPoseEstimate();
        }
        for (Component component : components) {
            component.update();
        }
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
        if(!isBusy() && actionQueue.size() != 0) actionQueue.pop().run();
    }

    public void waitForIdle() {
        waitForIdle(() -> {
        });
    }

    public void waitForActionsCompleted() {
        while (shooter.powerShotsController.getRunning() || isBusy() || !shooter.turret.isIdle() || shooter.magazine.getState() != Magazine.State.DOWN ||
                shooter.gunner.getState() != Gunner.State.IDLE || wobbleArm.getState() == WobbleArm.State.MACRO) {
            update();
        }
    }

    private void waitForIdle(Runnable block) {
        while (!Thread.currentThread().isInterrupted() && isBusy() && linearOpMode.opModeIsActive()) {
            block.run();
            update();
        }
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();// || shooter.gunner.getState() != Gunner.State.IDLE;
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
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS).firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface

        return (double) imu.getAngularVelocity().yRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}