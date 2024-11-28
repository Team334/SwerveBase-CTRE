package frc.robot.utils;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;

// this is in a seperate file because of the issue mentioned in the readme

/** Represents a single vision pose estimate. */
@Logged
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
   * Returns a vision pose estimate that represents an estimate with no detected tags (or camera was
   * disconnected).
   */
  public static final VisionPoseEstimate noDetectedTags() {
    return new VisionPoseEstimate(
        new Pose3d(), -1, -1, new int[0], -1, new double[] {-1, -1, -1}, false);
  }
}
