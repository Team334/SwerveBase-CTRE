package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class HolonomicController {
  private final ProfiledPIDController _xProfiled =
      new ProfiledPIDController(0, 0, 0, new Constraints(2, 0.5));
  private final ProfiledPIDController _yProfiled =
      new ProfiledPIDController(0, 0, 0, new Constraints(2, 0.5));
  private final ProfiledPIDController _headingProfiled =
      new ProfiledPIDController(0, 0, 0, new Constraints(2, 1));

  private final PIDController _xController = new PIDController(0, 0, 0);
  private final PIDController _yController = new PIDController(0, 0, 0);
  private final PIDController _headingController = new PIDController(0, 0, 0);

  /** Set the error tolerance for all controllers. */
  public void setTolerance(Transform2d tolerance) {
    _xProfiled.setTolerance(tolerance.getX());
    _yProfiled.setTolerance(tolerance.getY());
    _headingProfiled.setTolerance(tolerance.getRotation().getRadians());

    _xController.setTolerance(tolerance.getX());
    _yController.setTolerance(tolerance.getY());
    _headingController.setTolerance(tolerance.getRotation().getRadians());
  }

  /**
   * Whether the error between robot pose and goal (since the last {@link #calculate(Pose2d,
   * Pose2d)} call) is within the set tolerance or not.
   */
  public boolean atGoal() {
    return _xProfiled.atGoal() && _yProfiled.atGoal() && _headingProfiled.atGoal();
  }

  /**
   * Whether the error between robot pose and setpoint (since the last {@link
   * #calculate(ChassisSpeeds, Pose2d, Pose2d)} call) is within the tolerance or not.
   */
  public boolean atSetpoint() {
    return _xController.atSetpoint()
        && _yController.atSetpoint()
        && _headingController.atSetpoint();
  }

  /** Resets the motion profile at the current drive pose and chassis speeds. */
  public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    _xProfiled.reset(currentPose.getX(), currentSpeeds.vxMetersPerSecond);
    _yProfiled.reset(currentPose.getY(), currentSpeeds.vyMetersPerSecond);
    _headingProfiled.reset(
        currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Samples the motion profiles at the next timestep. The motion profiles end at the desired goal
   * pose at a chassis speeds of 0.
   *
   * @param currentPose The current pose of the chassis necessary for PID.
   * @param goalPose The desired goal pose (end of motion profiles) of the chassis.
   * @return Chassis speeds (including PID correction) sampled from the trapezoid profile at the
   *     next timestep.
   */
  public ChassisSpeeds calculate(Pose2d currentPose, Pose2d goalPose) {
    return new ChassisSpeeds(
        _xProfiled.calculate(currentPose.getX(), goalPose.getX()),
        _yProfiled.calculate(currentPose.getY(), goalPose.getY()),
        _headingProfiled.calculate(
            currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians()));
  }

  /**
   * Modifies some reference chassis speeds the drive is currently traveling at to bring the drive
   * closer to a desired pose.
   *
   * @param currentSpeeds The field-relative reference speeds the drive is traveling at.
   * @param desiredPose The desired pose.
   * @param currentPose The current pose of the drive.
   * @return New modified speeds.
   */
  public ChassisSpeeds calculate(
      ChassisSpeeds currentSpeeds, Pose2d desiredPose, Pose2d currentPose) {
    return currentSpeeds.plus(
        new ChassisSpeeds(
            _xController.calculate(currentPose.getX(), desiredPose.getX()),
            _yController.calculate(currentPose.getY(), desiredPose.getY()),
            _headingController.calculate(
                currentPose.getRotation().getRadians(), desiredPose.getRotation().getRadians())));
  }
}
