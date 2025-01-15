package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;

public class HolonomicController {
  private final ProfiledPIDController _translationProfiled =
      new ProfiledPIDController(0, 0, 0, new Constraints(10, 2));
  private final ProfiledPIDController _headingProfiled =
      new ProfiledPIDController(0, 0, 0, new Constraints(2, 1));

  private final PIDController _translationController = new PIDController(0, 0, 0);
  private final PIDController _headingController = new PIDController(0, 0, 0);

  /**
   * Set the error tolerance for all controllers.
   *
   * @param translationTolerance Linear translation tolerance in meters.
   * @param headingTolerance Heading tolerance.
   */
  public void setTolerance(Distance translationTolerance, Rotation2d headingTolerance) {
    _translationProfiled.setTolerance(translationTolerance.in(Meters));
    _headingProfiled.setTolerance(headingTolerance.getRadians());

    _translationController.setTolerance(translationTolerance.in(Meters));
    _headingController.setTolerance(headingTolerance.getRadians());
  }

  /**
   * Whether the error between robot pose and goal (since the last {@link #calculate(Pose2d,
   * Pose2d)} call) is within the set tolerance or not.
   */
  public boolean atGoal() {
    return _translationProfiled.atGoal() && _headingProfiled.atGoal();
  }

  /**
   * Whether the error between robot pose and setpoint (since the last {@link
   * #calculate(ChassisSpeeds, Pose2d, Pose2d)} call) is within the tolerance or not.
   */
  public boolean atSetpoint() {
    return _translationController.atSetpoint() && _headingController.atSetpoint();
  }

  /** Resets the motion profile at the current drive pose and field-relative chassis speeds. */
  public void reset(Pose2d currentPose, Pose2d goalPose, ChassisSpeeds currentSpeeds) {
    Transform2d transform = currentPose.minus(goalPose);

    // vector where tail is at goal pose and head is at current pose
    Vector<N2> difference = VecBuilder.fill(transform.getX(), transform.getY());

    System.out.println(
        difference.dot(
                VecBuilder.fill(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond))
            / difference.norm());

    _translationProfiled.reset(
        difference.norm(),
        difference.dot(
                VecBuilder.fill(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond))
            / difference.norm());

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
    Transform2d transform = currentPose.minus(goalPose);

    // vector where tail is at goal pose and head is at current pose
    Vector<N2> difference = VecBuilder.fill(transform.getX(), transform.getY());

    DogLog.log("FARTS", difference.norm());

    double velMag = _translationProfiled.calculate(difference.norm(), 0);

    DogLog.log("PISS", velMag);

    Vector<N2> vel = difference.unit().times(_translationProfiled.getSetpoint().velocity);

    DogLog.log("Swerve/Goal Pose Distance", difference.norm());
    DogLog.log("Swerve/Goal Pose Distance Velocity", velMag);

    DogLog.log("Swerve/Goal Pose", goalPose);

    return new ChassisSpeeds(
        vel.get(0),
        vel.get(1),
        _headingController.calculate(
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
    Transform2d transform = desiredPose.minus(currentPose);

    // vector where tail is at current pose and head is at desired pose
    Vector<N2> difference = VecBuilder.fill(transform.getX(), transform.getY());

    // find linear speed scalar returned by PID and set the length of the difference vector to the
    // scalar
    // this is so velocity is pointing in the right direction
    Vector<N2> vel =
        difference.unit().times(_translationController.calculate(difference.norm(), 0));

    return currentSpeeds.plus(
        new ChassisSpeeds(
            vel.get(0),
            vel.get(1),
            _headingController.calculate(
                currentPose.getRotation().getRadians(), desiredPose.getRotation().getRadians())));
  }
}
