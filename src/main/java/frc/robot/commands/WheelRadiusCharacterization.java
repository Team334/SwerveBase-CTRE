// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FaultLogger;
import frc.lib.FaultsTable.FaultType;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;


public class WheelRadiusCharacterization extends Command {
  /** Creates a new WheelRadiusCharacterization. */
  private final Swerve _swerve;

  private double _lastGyroYaw = 0;
  private double _accumGyroYaw = 0;

  private double[] _initialWheelPositions;

  private double _wheelRadius;

  public WheelRadiusCharacterization(Swerve swerve) {
    _swerve = swerve;
    addRequirements(_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _lastGyroYaw = _swerve.getHeading().getRadians();
    _accumGyroYaw = 0;

    _initialWheelPositions = _swerve.getWheelPositionsRadians();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _swerve.drive(0, 0, 1); // Make this tunable

    _accumGyroYaw += MathUtil.angleModulus(_swerve.getHeading().getRadians() - _lastGyroYaw);
    _lastGyroYaw = _swerve.getHeading().getRadians();

    double averageWheelPosition = 0;
    double[] wheelPositions = _swerve.getWheelPositionsRadians();

    for(int i = 0; i < _swerve.getModules().length; i++){
      averageWheelPosition += Math.abs(wheelPositions[i] - _initialWheelPositions[i]);
    }

    averageWheelPosition /= 4;

    _wheelRadius = (_accumGyroYaw + SwerveConstants.driveRadius.magnitude()) / averageWheelPosition;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _swerve.drive(0, 0, 0);

    if (_accumGyroYaw <= Math.PI * 2){
      FaultLogger.report("Need more info for characterization!", FaultType.ERROR);
    }
    else{
      System.out.println("Wheel Radius (inches): " + Units.metersToInches(_wheelRadius));
    }
  }
}
