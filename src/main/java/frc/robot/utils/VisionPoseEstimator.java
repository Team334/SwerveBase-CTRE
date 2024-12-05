// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** Handles pose estimation coming from a single PhotonVision camera. */
@Logged(strategy = Strategy.OPT_IN)
public class VisionPoseEstimator implements AutoCloseable {
  /** The camera's NT name. */
  @Logged(name = "Camera Name")
  public final String camName;

  /**
   * The ambiguity threshold on this camera to limit incoming vision estimates (used for filtering).
   */
  @Logged(name = "Ambiguity Threshold")
  public final double ambiguityThreshold;

  /**
   * Std devs factor based on this specific camera, increase it if the resolution is lowered on this
   * camera, if the fov is high, if the ambiguity threshold is increased, etc.
   */
  @Logged(name = "Camera Std Devs Factor")
  public final double cameraStdDevsFactor;

  /** The location of the camera relative to the robot's center. */
  @Logged(name = "Robot To Camera Transform")
  public final Transform3d robotToCam;

  /**
   * Whether this estimator is ignoring the vision heading estimate (if this is true the vision
   * theta std devs will be super high).
   */
  @Logged(name = "Ignore Theta Estimate")
  public boolean ignoreThetaEstimate = true;

  private final PhotonCamera _camera;
  private final PhotonPoseEstimator _poseEstimator;

  private final String _logPath;

  // new estimates from last update call
  private List<VisionPoseEstimate> _newEstimates = new ArrayList<>();

  /** Constants for a single vision pose estimator camera. */
  public record VisionPoseEstimatorConstants(
      /** The NT name of the camera. */
      String camName,

      /** The robot to camera transform */
      Transform3d robotToCam,

      /** The ambiguity threshold for filtering */
      double ambiguityThreshold,

      /** The camera's std devs factor. */
      double cameraStdDevsFactor) {}

  /** Represents a single vision pose estimate. */
  public record VisionPoseEstimate(
      /** The pose to add into the estimator. */
      Pose3d pose,

      /** The timestamp of when the frame was taken. (-1 when no tags). */
      double timestamp,

      /** The ambiguity of this measurement (-1 when no tags or when multi-tag). */
      double ambiguity,

      /** The detected tag ids in this measurement. */
      int[] detectedTags,

      /** The average distance from the tag(s) (-1 when no tags). */
      double avgTagDistance,

      /**
       * The [xMeters, yMeters, thetaRadians] noise standard deviations of this pose estimate ([-1,
       * -1, -1] when no tags or invalid).
       */
      double[] stdDevs,

      /** Whether this estimate passed the filter or not. */
      boolean isValid) {
    /**
     * Used for sorting a list of vision pose estimates, first the timestamps are sorted (from
     * smallest to highest), then the standard deviations at the same timestamp are sorted if
     * necessary.
     */
    public static final Comparator<VisionPoseEstimate> comparator =
        Comparator.comparing(
                VisionPoseEstimate::timestamp,
                (t1, t2) -> {
                  if (t1 > t2) return 1;
                  if (t1 < t2) return -1;
                  return 0;
                })
            .thenComparing(
                // this only happens when measurements land on the same timestamp, they need to be
                // sorted by decreasing std devs
                // this is further explained on the 3rd bullet point here:
                // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2023-build-thread/420691/36
                VisionPoseEstimate::stdDevs,
                (s1, s2) -> {
                  return -Double.compare(
                      // compare total s1 std devs to total s2 std devs
                      // if s1 (total) is greater than s2 (total), that actually means that we want
                      // s2 to come
                      // after s1 in the sorted array, which is why the negative symbol is needed
                      s1[0] + s1[1] + s1[2], s2[0] + s2[1] + s2[2]);
                });
  }

  /** Builds a new vision pose estimator from a single camera constants. */
  public static VisionPoseEstimator buildFromConstants(VisionPoseEstimatorConstants camConstants) {
    return new VisionPoseEstimator(
        camConstants.camName,
        camConstants.robotToCam,
        camConstants.ambiguityThreshold,
        camConstants.cameraStdDevsFactor);
  }

  /** Creates a new VisionPoseEstimator (all params are members that are javadocced already). */
  public VisionPoseEstimator(
      String camName,
      Transform3d robotToCam,
      double ambiguityThreshold,
      double cameraStdDevsFactor) {
    this.camName = camName;
    this.robotToCam = robotToCam;
    this.ambiguityThreshold = ambiguityThreshold;
    this.cameraStdDevsFactor = cameraStdDevsFactor;

    _camera = new PhotonCamera(camName);

    _poseEstimator =
        new PhotonPoseEstimator(
            FieldConstants.fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    // this is actually "closest-to-gyro" in the robot code
    _poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    _logPath = "Swerve/" + camName + "/Estimate/";
  }

  /**
   * Returns an array of the new estimates since the last {@link #update} call. This should be used
   * by the wpilib pose estimator.
   */
  public List<VisionPoseEstimate> getNewEstimates() {
    return _newEstimates;
  }

  // appends a new estimate to the log file
  private void logNewEstimate(VisionPoseEstimate estimate) {
    DogLog.log(_logPath + "Pose", estimate.pose);
    DogLog.log(_logPath + "Timestamp", estimate.timestamp);
    DogLog.log(_logPath + "Ambiguity", estimate.ambiguity);
    DogLog.log(_logPath + "Detected Tags", estimate.detectedTags);
    DogLog.log(_logPath + "Average Tag Distance", estimate.avgTagDistance);
    DogLog.log(_logPath + "Std Devs", estimate.stdDevs);
    DogLog.log(_logPath + "Is Valid", estimate.isValid);
  }

  /**
   * Processes a given {@link EstimatedRobotPose}, converting it into a filtered {@link
   * VisionPoseEstimate} with calculated measurement standard deviations.
   *
   * @param estimate The photon vision estimate.
   * @param gyroHeading The gyro heading at the given estimate timestamp (necessary for
   *     disambiguation).
   * @return A new vision pose estimate.
   */
  private VisionPoseEstimate processEstimate(EstimatedRobotPose estimate, Rotation2d gyroHeading) {
    // estimate properties
    Pose3d estimatedPose = estimate.estimatedPose;
    double timestamp = estimate.timestampSeconds;
    double ambiguity = -1;
    int tagAmount = estimate.targetsUsed.size();
    int[] detectedTags = new int[tagAmount];
    double avgTagDistance = 0;
    double[] stdDevs = new double[3];
    boolean isValid = false;

    // ---- DISAMBIGUATE (if single-tag) ----
    // disambiguate poses using gyro measurement (only necessary for a single tag)
    if (tagAmount == 1) {
      var target = estimate.targetsUsed.get(0);
      int tagId = target.getFiducialId();
      Pose3d tagPose = FieldConstants.fieldLayout.getTagPose(tagId).get();

      ambiguity = target.getPoseAmbiguity();

      Pose3d betterReprojPose = tagPose.transformBy(target.getBestCameraToTarget().inverse());
      Pose3d worseReprojPose = tagPose.transformBy(target.getAlternateCameraToTarget().inverse());

      betterReprojPose = betterReprojPose.transformBy(robotToCam.inverse());
      worseReprojPose = worseReprojPose.transformBy(robotToCam.inverse());

      // check which of the poses is closer to the correct gyro heading
      if (Math.abs(betterReprojPose.toPose2d().getRotation().minus(gyroHeading).getDegrees())
          < Math.abs(worseReprojPose.toPose2d().getRotation().minus(gyroHeading).getDegrees())) {
        estimatedPose = betterReprojPose;
      } else {
        estimatedPose = worseReprojPose;
      }
    }

    // ---- FILTER ----
    // get tag distance
    for (int i = 0; i < tagAmount; i++) {
      int tagId = estimate.targetsUsed.get(i).getFiducialId();
      Pose3d tagPose = FieldConstants.fieldLayout.getTagPose(tagId).get();
      detectedTags[i] = tagId;
      avgTagDistance += tagPose.getTranslation().getDistance(estimatedPose.getTranslation());
    }

    avgTagDistance /= tagAmount;

    // run all filtering
    boolean badAmbiguity = ambiguity >= ambiguityThreshold;
    boolean outOfBounds =
        (estimatedPose.getX() <= -VisionConstants.xBoundMargin
            || estimatedPose.getX()
                >= FieldConstants.fieldLayout.getFieldLength() + VisionConstants.xBoundMargin
            || estimatedPose.getY() <= -VisionConstants.yBoundMargin
            || estimatedPose.getY()
                >= FieldConstants.fieldLayout.getFieldWidth() + VisionConstants.yBoundMargin
            || estimatedPose.getZ() >= VisionConstants.zBoundMargin
            || estimatedPose.getZ() <= -VisionConstants.zBoundMargin);

    isValid = !(badAmbiguity || outOfBounds);

    // ---- STD DEVS CALCULATION ----
    if (isValid) {
      double[] baseStdDevs =
          detectedTags.length == 1
              ? VisionConstants.singleTagBaseStdDevs
              : VisionConstants.multiTagBaseStdDevs;

      double xStdDevs = baseStdDevs[0] * Math.pow(avgTagDistance, 2) * cameraStdDevsFactor;
      double yStdDevs = baseStdDevs[1] * Math.pow(avgTagDistance, 2) * cameraStdDevsFactor;
      double thetaStdDevs = baseStdDevs[2] * Math.pow(avgTagDistance, 2) * cameraStdDevsFactor;

      if (ignoreThetaEstimate) thetaStdDevs = 999999999;

      stdDevs[0] = xStdDevs;
      stdDevs[1] = yStdDevs;
      stdDevs[2] = thetaStdDevs;
    }

    return new VisionPoseEstimate(
        estimatedPose, timestamp, ambiguity, detectedTags, avgTagDistance, stdDevs, isValid);
  }

  /** Reads from the camera and generates an array of new latest {@link VisionPoseEstimate}(s). */
  public void update() {
    for (var estimate : _camera.getAllUnreadResults()) {
      var est = _poseEstimator.update(estimate);

      if (est.isPresent()) {
        var newEstimate = processEstimate(est.get(), Rotation2d.kZero); // TODO: use the actual gyro
        _newEstimates.add(newEstimate);

        logNewEstimate(newEstimate);
      }
    }
  }

  @Override
  public void close() throws Exception {
    _camera.close();
  }
}
