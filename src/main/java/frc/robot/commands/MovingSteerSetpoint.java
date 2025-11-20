// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// to delete later
public class MovingSteerSetpoint extends Command {
  private final Swerve swerve;

  private PointWheelsAt pwa = new PointWheelsAt();

  private double d = 0;

  /** Creates a new MovingSteerSetpoint. */
  public MovingSteerSetpoint(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);

    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    d = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    d += 3;

    swerve.setControl(pwa.withModuleDirection(Rotation2d.fromDegrees(d)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
