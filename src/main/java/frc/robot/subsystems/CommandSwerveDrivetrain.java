// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  /**
   * Creates a new CommandSwerveDrivetrain.
   * 
   * @param driveTrainConstants The CTRE {@link SwerveDrivetrainConstants}. These involve the CAN Bus name and the Pigeon Id.
   * @param moduleConstants The CTRE {@link SwerveModuleConstants}. The involve constants identical across all modules (PID constants, 
   * wheel radius, etc), and constants unique to each module (location, device ids, etc).
   */
  public CommandSwerveDrivetrain(
    SwerveDrivetrainConstants drivetrainConstants,
    SwerveModuleConstants... moduleConstants
  ) {
    super(drivetrainConstants, moduleConstants);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
