package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class HolonomicController {
  private final TrapezoidProfile _xProfile = new TrapezoidProfile(null);
  private final TrapezoidProfile _yProfile = new TrapezoidProfile(null);
  private final TrapezoidProfile _headingProfile = new TrapezoidProfile(null);

  private final PIDController _xController = new PIDController(0, 0, 0);
  private final PIDController _yController = new PIDController(0, 0, 0);
  private final PIDController _headingController = new PIDController(0, 0, 0);

  private Pose2d _error = new Pose2d();
  private Pose2d _setpoint = new Pose2d();

  private Pose2d _tolerance = new Pose2d();

  public void setTolerance(Pose2d tolerance) {
    _tolerance = tolerance;
  }

  public void setSetpoint(Pose2d setpoint) {
    _setpoint = setpoint;
  }

  /** A state the drive can be at. */
  public record State(Pose2d pose, ChassisSpeeds speeds) {}

  /**
   * Whether the error of the holonomic controller (since the last {@link #calculate call}) is
   * within the tolerance or not.
   */
  public boolean atReference() {
    return MathUtil.isNear(_setpoint.getX(), _error.getX(), _tolerance.getX())
        && MathUtil.isNear(_setpoint.getY(), _error.getY(), _tolerance.getY())
        && MathUtil.isNear(
            _setpoint.getRotation().getRadians(),
            _error.getRotation().getRadians(),
            _tolerance.getRotation().getRadians());
  }

  public ChassisSpeeds calculate(State current, State goal, double t) {
    return new ChassisSpeeds();
  }
}
