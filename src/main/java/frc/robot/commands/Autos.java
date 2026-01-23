package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import frc.robot.subsystems.Swerve;

/** All auton routines. */
public class Autos {
  private final AutoFactory _factory;

  private final Swerve _swerve;

  public Autos(Swerve swerve) {
    _swerve = swerve;

    _factory =
        new AutoFactory(
            _swerve::getPose,
            _swerve::resetPose,
            _swerve::followTrajectory,
            true,
            _swerve,
            (traj, isActive) -> {
              DogLog.log("Auto/Current Trajectory", traj.getPoses());
              DogLog.log("Auto/Current Trajectory Name", traj.name());
              DogLog.log("Auto/Current Trajectory Duration", traj.getTotalTime());
              DogLog.log("Auto/Current Trajectory Is Active", isActive);
            });
  }

  public AutoRoutine example() {
    AutoRoutine routine = _factory.newRoutine("example");

    AutoTrajectory exampleTraj = routine.trajectory("example");

    routine.active().onTrue(sequence(exampleTraj.resetOdometry(), exampleTraj.cmd()));

    return routine;
  }

  public AutoRoutine mrc() {
    AutoRoutine routine = _factory.newRoutine("mrc");

    AutoTrajectory traj = routine.trajectory("mrc");

    routine.active().onTrue(sequence(traj.resetOdometry(), traj.cmd()));

    return routine;
  }

  public AutoRoutine brotate() {
    AutoRoutine routine = _factory.newRoutine("brotate");

    AutoTrajectory traj = routine.trajectory("brotate");

    routine.active().onTrue(sequence(traj.resetOdometry(), traj.cmd()));

    return routine;
  }

  public AutoRoutine vocim() {
    AutoRoutine routine = _factory.newRoutine("vocim");

    AutoTrajectory traj = routine.trajectory("vocim");

    routine.active().onTrue(sequence(traj.resetOdometry(), traj.cmd()));

    return routine;
  }

  public AutoRoutine square() {
    AutoRoutine routine = _factory.newRoutine("square");

    AutoTrajectory traj = routine.trajectory("square");

    routine.active().onTrue(sequence(traj.resetOdometry(), traj.cmd()));

    return routine;
  }
}
