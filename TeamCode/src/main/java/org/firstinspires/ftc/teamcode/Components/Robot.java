package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Components.localizer.t265Localizer;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.*;

public class Robot extends MecanumDrive {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final int HORIZON = 100; // horizon value to tune
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam
    public OpenCvCamera webcam;
    private UGContourRingPipeline pipeline;
    private RingLocalizer bouncebacks;

    public static OpModeType opModeType = OpModeType.NONE;
    public static Side side = Side.RED;
    public UGContourRingPipeline.Height height = UGContourRingPipeline.Height.ZERO;

    public final LinearOpMode linearOpMode;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public Intake intake;
    private final DcMotorEx leftFront, leftRear, rightRear, rightFront;

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

    public static Pose2d robotPose = new Pose2d();
    public static Vector2d rightWobble = new Vector2d(-32, -51);

    public Shooter shooter;
    public WobbleArm wobbleArm;
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public enum Mode {
        IDLE,
        TURN,
    }

    private final FtcDashboard dashboard;
    private final NanoClock clock;
    private Mode mode;
    private final PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);
    private TrajectorySequenceRunner trajectorySequenceRunner;
    private final TrajectoryFollower follower;
    private final LinkedList<Pose2d> poseHistory;
    private final List<DcMotorEx> motors;
    private final BNO055IMU imu;
    private final VoltageSensor batteryVoltageSensor;
    private Pose2d lastPoseOnTurn;
    LinkedList<Runnable> actionQueue = new LinkedList<Runnable>();
    public Robot(LinearOpMode opMode, Pose2d pose2d) {
        this(opMode, pose2d, OpModeType.NONE);
    }
    public Robot(LinearOpMode opMode, OpModeType type){
        this(opMode, robotPose, type);
    }
    public Robot(LinearOpMode opMode) {
        this(opMode, new Pose2d(0,0,0), OpModeType.NONE);
    }
    public Robot(LinearOpMode opMode, Pose2d pose2d, OpModeType type) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        opModeType = type;
        robotPose = pose2d.plus(new Pose2d(-70.4725, -70.4725, 0));
        linearOpMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        clock = NanoClock.system();
        mode = Mode.IDLE;
        turnController = new PIDFController(new PIDCoefficients(7, 0, 0));
        turnController.setInputBounds(0, 2 * Math.PI);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
        poseHistory = new LinkedList<>();
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
        setLocalizer(new t265Localizer(hardwareMap));
        setPoseEstimate(robotPose);
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
        switch (side) {
            case RED: break;
            case BLUE: break;
        }
    }
    public void autoInit(){
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
    public Trajectory pickup;
    public void createTraj(double endTangent){
        TrajectoryBuilder builder = trajectoryBuilder(Coordinate.toPose(powerShotLocals[2], 0))
                .splineTo(new Vector2d(15, 7), new Vector2d(10, 8).angleBetween(new Vector2d(42, 9)))
                .splineTo(new Vector2d(42, 7), 0);
        Vector2d[] wayPoints = bouncebacks.getVectors(getPoseEstimate()).toArray(new Vector2d[0]).clone();
        bubbleSort(wayPoints, new Vector2d());
        for(int i = 0; i < wayPoints.length; i++){
            builder = builder.splineTo(wayPoints[i], i < wayPoints.length - 2 ? wayPoints[i].angleBetween(wayPoints[i + 1]) : endTangent);
        }
        pickup = builder.splineTo(new Vector2d(55.7, -38), Math.toRadians(-75))
                .build();
    }
    void bubbleSort(Vector2d[] arr, Vector2d referencePose) {
        int n = arr.length;
        for (int i = 0; i < n-1; i++)
            for (int j = 0; j < n-i-1; j++)
                if (arr[j].distTo(referencePose) > arr[j+1].distTo(referencePose))
                {
                    // swap arr[j+1] and arr[j]
                    Coordinate temp = Coordinate.toPoint(arr[j]);
                    arr[j] = arr[j+1];
                    arr[j+1] = temp.toVector();
                }
    }
    public void switchPipeline(){
        webcam.setPipeline(bouncebacks = new RingLocalizer());
    }
    public void scan(){
        height = pipeline.getHeight();
    }
    public void turnOffVision(){
        webcam.closeCameraDeviceAsync(()-> webcam.stopStreaming());
        dashboard.stopCameraStream();
    }
    public void wobbleArmUp() {
        wobbleArm.dropMacro();
    }
    public void wobbleArmDown() {
        wobbleArm.dropMacro();

    }
    public void wobbleArmVertical(){
        wobbleArm.dropMacro();
    }
    public void grab(){
        wobbleArm.claw.grab();
    }
    public void release(){
        wobbleArm.claw.release();
    }
    public void intake(double intakeSpeed){
        intake.setPower(-intakeSpeed);
    }
    public void optimalShoot(){
        shooter.tripleShot();
    }
    public void shieldUp() {
        intake.shieldUp();
    }
    public void shieldDown(){
        intake.shieldDown();
    }
    public Vector2d getDropZone(){
        if(height == UGContourRingPipeline.Height.FOUR){
            return dropZones[2];
        } else if(height == UGContourRingPipeline.Height.ONE){
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
        return new TrajectoryBuilder(Robot.robotPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );

        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void turn(double angle, Runnable runnable){
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

    public void update() {
        updatePoseEstimate();
        shooter.update();
        wobbleArm.update();
        intake.update();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
        //if(!isBusy()) actionQueue.pop().run();
    }
    public void waitForIdle() {
        waitForIdle(()->{});
    }
    public void waitForActionsCompleted() {
        while(shooter.powerShotsController.getRunning() || isBusy() || !shooter.turret.isIdle() || shooter.magazine.getState() != Magazine.State.DOWN ||
                shooter.gunner.getState() != Gunner.State.IDLE || wobbleArm.getState() == WobbleArm.State.TRANSIT){
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
        return mode != Mode.IDLE || shooter.gunner.getState() != Gunner.State.IDLE;
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

    public void setMode(Mode mode){
        this.mode = mode;
    }

    public Mode getMode(){
        return mode;
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