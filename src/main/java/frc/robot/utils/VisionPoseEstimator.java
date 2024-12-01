// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;

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

  // the latest vision estimate from the pose estimator
  @Logged(name = "Latest Estimate")
  private VisionPoseEstimate _latestEstimate = VisionPoseEstimate.noDetectedTags();

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

  /** Builds a new vision pose estimator from a single camera constants. */
  public static VisionPoseEstimator buildFromCamera(VisionPoseEstimatorConstants camConstants) {
    return new VisionPoseEstimator(
        camConstants.camName,
        camConstants.robotToCam,
        camConstants.ambiguityThreshold,
        camConstants.cameraStdDevsFactor);
  }

  /**
   * Builds multiple vision pose estimators from a list of camera constants.
   *
   * @return An array of estimators.
   */
  public static List<VisionPoseEstimator> buildFromCameras(
      List<VisionPoseEstimatorConstants> allConstants) {
    List<VisionPoseEstimator> estimators = new ArrayList<VisionPoseEstimator>();

    allConstants.forEach(
        camConstants -> {
          estimators.add(buildFromCamera(camConstants));
        });

    return estimators;
  }

  /**
   * Builds an empty array of pose estimators. Use this if there are no cameras on the robot.
   *
   * @return An empty array of pose estimators.
   */
  public static List<VisionPoseEstimator> noCameras() {
    return new ArrayList<VisionPoseEstimator>();
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
  }

  @Override
  public void close() throws Exception {}
}
