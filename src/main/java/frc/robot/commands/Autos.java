// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.auto.AutoRoutine;
import dev.doglog.DogLog;
import choreo.auto.AutoTrajectory;
import frc.robot.subsystems.Swerve;

public class Autos {
  private final Swerve _swerve;

  private final AutoFactory _factory;

  public Autos(Swerve swerve) {
    _swerve = swerve;

    _factory =
        new AutoFactory(
            _swerve::getPose,
            _swerve::resetPose,
            _swerve::followTrajectory,
            true,
            _swerve,
            new AutoBindings(), // TODO
            (traj, isActive) -> {
              traj = traj.flipped();

              DogLog.log("Auto/Current Trajectory", traj.getPoses());
              DogLog.log("Auto/Current Trajectory Name", traj.name());
              DogLog.log("Auto/Current Trajectory Duration", traj.getTotalTime());
              DogLog.log("Auto/Current Trajectory Is Active", isActive);
            });
  }

  public AutoRoutine simpleTrajectory() {
    var routine = _factory.newRoutine("Routine");
    var traj = routine.trajectory("Simple Trajectory");

    routine.active().onTrue(sequence(routine.resetOdometry(traj), traj.cmd()));

    return routine;
  }

  public AutoRoutine branchingAuto(){
    final boolean _branch = true; // Change this if you want the auton to branch (Maybe simulate some trigger later)

    AutoRoutine routine = _factory.newRoutine("branchingAuto");

    // Loading traj
    AutoTrajectory startToM1 = routine.trajectory("startToM1");
    AutoTrajectory M1toScore = routine.trajectory("M1toScore");
    AutoTrajectory M1toM2 = routine.trajectory("M1toM2");
    AutoTrajectory M2toScore = routine.trajectory("M2toScore");

    routine.active().onTrue(sequence(routine.resetOdometry(startToM1), startToM1.cmd()));

    startToM1.done().and(() -> _branch).onTrue(M1toM2.cmd());
    startToM1.done().and(() -> !_branch).onTrue(M1toScore.cmd());

    M1toM2.done().onTrue(M2toScore.cmd());

    return routine;
  }
}
