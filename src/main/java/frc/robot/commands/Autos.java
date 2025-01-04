// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
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
            new AutoBindings() // TODO
            );
  }

  // TODO: add a simple path example
}
