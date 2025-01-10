package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class HolonomicController {
  // TODO: make a method for profiled control
  private final TrapezoidProfile _translationProfile = new TrapezoidProfile(null);
  private final TrapezoidProfile _headingProfile = new TrapezoidProfile(null);

  private final PIDController _translationController = new PIDController(0, 0, 0);
  private final PIDController _headingController = new PIDController(0, 0, 0);

  private Transform2d _error = new Transform2d();
  private Pose2d _setpoint = new Pose2d();

  private Transform2d _tolerance = new Transform2d();

  public void setTolerance(Transform2d tolerance) {
    _tolerance = tolerance;
  }

  public Transform2d getTolerance() {
    return _tolerance;
  }

  public void setSetpoint(Pose2d setpoint) {
    _setpoint = setpoint;
  }

  public Pose2d getSetpoint() {
    return _setpoint;
  }

  /**
   * Whether the error of the holonomic controller (since the last {@link #calculate call}) is
   * within the tolerance or not.
   */
  public boolean atReference() {
    return MathUtil.isNear(0, _error.getX(), _tolerance.getX())
        && MathUtil.isNear(0, _error.getY(), _tolerance.getY())
        && MathUtil.isNear(
            0, _error.getRotation().getRadians(), _tolerance.getRotation().getRadians());
  }

  /**
   * Modifies some reference chassis speeds the drive is currently traveling at to bring the drive
   * closer to a desired pose.
   *
   * @param referenceSpeeds The field-relative reference speeds the drive is traveling at.
   * @param desiredPose The desired pose.
   * @param referencePose The current reference pose of the drive.
   * @return Modified reference speeds.
   */
  public ChassisSpeeds calculate(
      ChassisSpeeds referenceSpeeds, Pose2d desiredPose, Pose2d referencePose) {
    // error is the transformation between the desired and reference pose
    _error = desiredPose.minus(referencePose);

    // vector where tail is at reference pose and head is at desired pose
    Vector<N2> difference = VecBuilder.fill(_error.getX(), _error.getY());

    // feed distance scalar into pid controller, and then construct a new velocity vector of length
    // of pid output
    // and direction of difference vector
    Vector<N2> vel =
        difference.unit().times(_translationController.calculate(difference.norm(), 0));

    referenceSpeeds.vxMetersPerSecond += vel.get(0);
    referenceSpeeds.vyMetersPerSecond += vel.get(1);

    referenceSpeeds.omegaRadiansPerSecond +=
        _headingController.calculate(
            referencePose.getRotation().getRadians(), desiredPose.getRotation().getRadians());

    return referenceSpeeds;
  }
}
