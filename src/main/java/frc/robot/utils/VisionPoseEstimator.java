// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.FieldConstants;
import java.util.ArrayList;
import java.util.List;
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
  public final boolean ignoreThetaEstimate = true;

  private final PhotonCamera _camera;
  private final PhotonPoseEstimator _poseEstimator;

  private final String _logPath;

  // new estimates from last update call
  private List<VisionPoseEstimate> _newEstimates = new ArrayList<VisionPoseEstimate>();

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
     * Returns a vision pose estimate that represents an estimate with no detected tags (or camera
     * was disconnected).
     */
    public static final VisionPoseEstimate noDetectedTags() {
      return new VisionPoseEstimate(
          new Pose3d(), -1, -1, new int[0], -1, new double[] {-1, -1, -1}, false);
    }
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

  // appends all the new estimates to the log file
  private void logNewEstimates() {
    _newEstimates.forEach(
        e -> {
          DogLog.log(_logPath + "Pose", e.pose);
          DogLog.log(_logPath + "Timestamp", e.timestamp);
          DogLog.log(_logPath + "Ambiguity", e.ambiguity);
          DogLog.log(_logPath + "Detected Tags", e.detectedTags);
          DogLog.log(_logPath + "Average Tag Distance", e.avgTagDistance);
          DogLog.log(_logPath + "Std Devs", e.stdDevs);
          DogLog.log(_logPath + "Is Valid", e.isValid);
        });
  }

  /**
   * Returns an array of the new estimates since the last {@link #update} call. This should be used
   * by the wpilib pose estimator.
   */
  public List<VisionPoseEstimate> getNewEstimates() {
    return _newEstimates;
  }

  /** Reads from the camera and generates an array of new latest {@link VisionPoseEstimate}(s). */
  public void update() {
    // TODO

    logNewEstimates();
  }

  @Override
  public void close() throws Exception {}
}
