package org.firstinspires.ftc.teamcode.trajectorysequenceimproved;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;

import org.firstinspires.ftc.teamcode.roadrunnerext.ImprovedTrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment.ConditionalWait;
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment.FutureSegment;
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment.WaitSegment;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;

import androidx.annotation.Nullable;

import static org.firstinspires.ftc.teamcode.util.field.Context.packet;


@Config
public class TrajectorySequenceRunner {
    public static String COLOR_INACTIVE_TRAJECTORY = "#4caf507a";
    public static String COLOR_INACTIVE_TURN = "#7c4dff7a";
    public static String COLOR_INACTIVE_WAIT = "#dd2c007a";

    public static String COLOR_INACTIVE_CONDITIONAL_WAIT = "#8D2377";
    public static String COLOR_ACTIVE_CONDITIONAL_WAIT = "#EA37C5";

    public static String COLOR_INACTIVE_FUTURE_SEGMENT = "#3B0381";
    public static String COLOR_ACTIVE_FUTURE_SEGMENT = "#761FE4";

    public static String COLOR_ACTIVE_TRAJECTORY = "#4CAF50";
    public static String COLOR_ACTIVE_TURN = "#7c4dff";
    public static String COLOR_ACTIVE_WAIT = "#dd2c00";

    public static int POSE_HISTORY_LIMIT = 100;

    private final ImprovedTrajectoryFollower follower;

    private final PIDFController turnController;

    private final NanoClock clock;
    private double offset;
    private ElapsedTime time = new ElapsedTime();

    private TrajectorySequence currentTrajectorySequence;
    private double currentSegmentStartTime;
    private int currentSegmentIndex;
    private int lastSegmentIndex;

    private Pose2d lastPoseError = new Pose2d();

    List<TrajectoryMarker> remainingMarkers = new ArrayList<>();

    private final FtcDashboard dashboard;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();
    private final PIDCoefficients headingPIDCoefficients;

    public TrajectorySequenceRunner(ImprovedTrajectoryFollower follower, PIDCoefficients headingPIDCoefficients) {
        this.follower = follower;
        this.headingPIDCoefficients = headingPIDCoefficients;

        turnController = new PIDFController(headingPIDCoefficients);
        turnController.setInputBounds(0, 2 * Math.PI);

        clock = NanoClock.system();

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        time.reset();
        currentTrajectorySequence = trajectorySequence;
        currentSegmentStartTime = clock.seconds();
        currentSegmentIndex = 0;
        lastSegmentIndex = -1;
    }

    public @Nullable
    DriveSignal update(Pose2d poseEstimate, Pose2d poseVelocity) {
        Pose2d targetPose = null;
        DriveSignal driveSignal = null;

        Canvas fieldOverlay = packet.fieldOverlay();

        SequenceSegment currentSegment = null;

        if (currentTrajectorySequence != null) {
            if (currentSegmentIndex >= currentTrajectorySequence.size()) {
                for (TrajectoryMarker marker : remainingMarkers) {
                    marker.getCallback().onMarkerReached();
                }

                remainingMarkers.clear();

                currentTrajectorySequence = null;
            }

            if (currentTrajectorySequence == null)
                return new DriveSignal();

            double now = clock.seconds();
            boolean isNewTransition = currentSegmentIndex != lastSegmentIndex;

            currentSegment = currentTrajectorySequence.get(currentSegmentIndex);

            if (isNewTransition) {
                if (lastSegmentIndex >= 0) {
                    if (currentTrajectorySequence.get(lastSegmentIndex) instanceof FutureSegment) {
                        offset += time.seconds();
                    }
                }
                time.reset();
                currentSegmentStartTime = now;
                lastSegmentIndex = currentSegmentIndex;

                for (TrajectoryMarker marker : remainingMarkers) {
                    marker.getCallback().onMarkerReached();
                }

                remainingMarkers.clear();

                remainingMarkers.addAll(currentSegment.getMarkers());
                remainingMarkers.sort(Comparator.comparingDouble(TrajectoryMarker::getTime));
            }

            double deltaTime = now - currentSegmentStartTime;

            if (currentSegment instanceof TrajectorySegment) {
                Trajectory currentTrajectory = ((TrajectorySegment) currentSegment).getTrajectory();

                if (isNewTransition)
                    follower.followTrajectory(currentTrajectory);

                if (!follower.isFollowing()) {
                    currentSegmentIndex++;

                    driveSignal = new DriveSignal();
                } else {
                    driveSignal = follower.update(poseEstimate, poseVelocity);
                    lastPoseError = follower.getLastError();
                }

                targetPose = currentTrajectory.get(deltaTime);
            } else if (currentSegment instanceof FutureSegment) {
                TrajectorySequenceRunner runner = new TrajectorySequenceRunner(follower, headingPIDCoefficients);
                runner.followTrajectorySequenceAsync(((FutureSegment) currentSegment).getTrajectory());
                if (!runner.follower.isFollowing()) {
                    currentSegmentIndex++;
                }
                return runner.update(poseEstimate, poseVelocity);
            } else if (currentSegment instanceof TurnSegment) {
                MotionState targetState = ((TurnSegment) currentSegment).getMotionProfile().get(deltaTime);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(poseEstimate.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();

                lastPoseError = new Pose2d(0, 0, turnController.getLastError());

                Pose2d startPose = currentSegment.getStartPose();
                targetPose = startPose.copy(startPose.getX(), startPose.getY(), targetState.getX());

                driveSignal = new DriveSignal(
                        new Pose2d(0, 0, targetOmega + correction),
                        new Pose2d(0, 0, targetAlpha)
                );

                if (deltaTime >= currentSegment.getDuration().invoke()) {
                    currentSegmentIndex++;
                    driveSignal = new DriveSignal();
                }
            } else if (currentSegment instanceof WaitSegment) {
                lastPoseError = new Pose2d();

                targetPose = currentSegment.getStartPose();
                driveSignal = ((WaitSegment) currentSegment).getDriveSignal();
                if (deltaTime >= currentSegment.getDuration().invoke()) {
                    currentSegmentIndex++;
                }
            } else if (currentSegment instanceof ConditionalWait) {
                lastPoseError = new Pose2d();

                targetPose = currentSegment.getStartPose();
                driveSignal = new DriveSignal();
                if (!((ConditionalWait) currentSegment).getCondition().invoke()) {
                    offset += time.seconds();
                    time.reset();
                    currentSegmentIndex++;
                }
            }

            while (!(currentSegment instanceof ConditionalWait) && remainingMarkers.size() > 0 && deltaTime > remainingMarkers.get(0).getTime() + offset) {
                remainingMarkers.get(0).getCallback().onMarkerReached();
                remainingMarkers.remove(0);
            }
        }

        poseHistory.add(poseEstimate);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        packet.put("x", poseEstimate.getX());
        packet.put("y", poseEstimate.getY());
        packet.put("heading (deg)", Math.toDegrees(poseEstimate.getHeading()));

        packet.put("xError", getLastPoseError().getX());
        packet.put("yError", getLastPoseError().getY());
        packet.put("headingError (deg)", Math.toDegrees(getLastPoseError().getHeading()));

        draw(fieldOverlay, currentTrajectorySequence, currentSegment, targetPose, poseEstimate);

        dashboard.sendTelemetryPacket(packet);

        packet = new TelemetryPacket();

        return driveSignal;
    }

    private void draw(
            Canvas fieldOverlay,
            TrajectorySequence sequence, SequenceSegment currentSegment,
            Pose2d targetPose, Pose2d poseEstimate
    ) {
        if (sequence != null) {
            for (int i = 0; i < sequence.size(); i++) {
                SequenceSegment segment = sequence.get(i);

                if (segment instanceof TrajectorySegment) {
                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_INACTIVE_TRAJECTORY);

                    DashboardUtil.drawSampledPath(fieldOverlay, ((TrajectorySegment) segment).getTrajectory().getPath());
                } else if (segment instanceof TurnSegment) {
                    Pose2d pose = segment.getStartPose();

                    fieldOverlay.setFill(COLOR_INACTIVE_TURN);
                    fieldOverlay.fillCircle(pose.getX(), pose.getY(), 2);
                } else if (segment instanceof WaitSegment) {
                    Pose2d pose = segment.getStartPose();

                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_INACTIVE_WAIT);
                    fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3);
                } else if (segment instanceof ConditionalWait) {
                    Pose2d pose = segment.getStartPose();

                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_INACTIVE_CONDITIONAL_WAIT);
                    fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3);
                } else if (segment instanceof FutureSegment) {
                    if (((FutureSegment) segment).getTrajectory() != null) {
                        fieldOverlay.setStrokeWidth(1);
                        fieldOverlay.setStroke(COLOR_INACTIVE_FUTURE_SEGMENT);
                        draw(fieldOverlay, ((FutureSegment) segment).getTrajectory(), currentSegment, targetPose, poseEstimate);
                    }
                }
            }
        }

        if (currentSegment != null) {
            if (currentSegment instanceof TrajectorySegment) {
                Trajectory currentTrajectory = ((TrajectorySegment) currentSegment).getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke(COLOR_ACTIVE_TRAJECTORY);

                DashboardUtil.drawSampledPath(fieldOverlay, currentTrajectory.getPath());
            } else if (currentSegment instanceof TurnSegment) {
                Pose2d pose = currentSegment.getStartPose();

                fieldOverlay.setFill(COLOR_ACTIVE_TURN);
                fieldOverlay.fillCircle(pose.getX(), pose.getY(), 3);
            } else if (currentSegment instanceof WaitSegment) {
                Pose2d pose = currentSegment.getStartPose();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke(COLOR_ACTIVE_WAIT);
                fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3);
            }  else if (currentSegment instanceof ConditionalWait) {
                Pose2d pose = currentSegment.getStartPose();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke(COLOR_ACTIVE_CONDITIONAL_WAIT);
                fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3);
            }
        }

        if (targetPose != null) {
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#4CAF50");
            DashboardUtil.drawRobot(fieldOverlay, targetPose);
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
    }

    public Pose2d getLastPoseError() {
        return lastPoseError;
    }

    public boolean isBusy() {
        return currentTrajectorySequence != null;
    }
}
