package frc.robot.utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HolonomicController {
  // used to follow the path
  private final PIDController _xController = new PIDController(0, 0, 0);
  private final PIDController _yController = new PIDController(0, 0, 0);

  private final PIDController _headingController = new PIDController(0, 0, 0);

  public HolonomicController() {
    _headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Modifies some base chassis speeds the drive is currently traveling at to bring the drive closer
   * to a desired pose.
   *
   * @param baseSpeeds The field-relative speed the drive is already traveling at.
   * @param desiredPose The desired pose.
   * @param currentPose The current pose of the drive.
   * @return New modified speeds.
   */
  public ChassisSpeeds calculate(ChassisSpeeds baseSpeeds, Pose2d desiredPose, Pose2d currentPose) {
    DogLog.log("Auto/Controller Desired Pose", desiredPose);
    DogLog.log("Auto/Controller Reference Pose", currentPose);

    return baseSpeeds.plus(
        new ChassisSpeeds(
            _xController.calculate(currentPose.getX(), desiredPose.getX()),
            _yController.calculate(currentPose.getY(), desiredPose.getY()),
            _headingController.calculate(
                currentPose.getRotation().getRadians(), desiredPose.getRotation().getRadians())));
  }
}
