// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.InputStream;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  // teleop requests
  private final RobotCentric _robotCentricRequest = new RobotCentric();
  private final FieldCentric _fieldCentricRequest = new FieldCentric();

  // auton request
  private final ApplyRobotSpeeds _robotSpeedsRequest = new ApplyRobotSpeeds();

  private double _lastSimTime = 0;
  private Notifier _simNotifier;


  private final ChassisSpeeds _driverChassisSpeeds = new ChassisSpeeds();

  private boolean _isFieldOriented = true;
  private boolean _isOpenLoop = true;

  /**
   * Creates a new CommandSwerveDrivetrain.
   * 
   * @param drivetrainConstants The CTRE {@link SwerveDrivetrainConstants}. These involve the CAN Bus name and the Pigeon Id.
   * @param moduleConstants The CTRE {@link SwerveModuleConstants}. The involve constants identical across all modules (PID constants, 
   * wheel radius, etc), and constants unique to each module (location, device ids, etc).
   */
  public CommandSwerveDrivetrain(
    SwerveDrivetrainConstants drivetrainConstants,
    SwerveModuleConstants... moduleConstants
  ) {
    super(drivetrainConstants, SwerveConstants.odometryFrequency.in(Hertz), moduleConstants);

    _robotCentricRequest.withDeadband(SwerveConstants.translationalDeadband)
        .withRotationalDeadband(SwerveConstants.rotationalDeadband);
    
    _fieldCentricRequest.withDeadband(SwerveConstants.translationalDeadband)
        .withRotationalDeadband(SwerveConstants.rotationalDeadband);

    _robotSpeedsRequest.withDriveRequestType(DriveRequestType.Velocity);

    if (RobotBase.isSimulation()) startSimThread();
  }

  /**
   * Creates a new Command that drives the drive. This is meant for teleop.
   * 
   * @param velX The x velocity in meters per second.
   * @param velY The y velocity in meters per second.
   * @param velOmega The rotational velocity in radians per second.
   */
  public Command drive(InputStream velX, InputStream velY, InputStream velOmega) {
    return run(() -> {
      drive(
        velX.get(),
        velY.get(),
        velOmega.get()
      );
    }).beforeStarting(() -> {
      _isFieldOriented = true;
      _isOpenLoop = true;
    }).withName("Drive");
  }


  /** 
   * Drives the swerve drive. This is meant for teleop.
   * 
   * @param velX The x velocity in meters per second. 
   * @param velY The y velocity in meters per second.
   * @param velOmega The rotational velocity in radians per second.
   */
  public void drive(double velX, double velY, double velOmega) {
    _driverChassisSpeeds.vxMetersPerSecond = velX;
    _driverChassisSpeeds.vyMetersPerSecond = velY;
    _driverChassisSpeeds.omegaRadiansPerSecond = velOmega;

    if (_isFieldOriented) {
      setControl(
          _fieldCentricRequest.withVelocityX(velX)
              .withVelocityY(velY)
              .withRotationalRate(velOmega)
              .withDriveRequestType(_isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity));
    } else {
      setControl(
          _robotCentricRequest.withVelocityX(velX)
              .withVelocityY(velY)
              .withRotationalRate(velOmega)
              .withDriveRequestType(_isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity));
    }
  }

  /**
   * Drives the swerve drive. This configuration is robot oriented and closed-loop, specifically meant for auton.
   * 
   * @param speeds The robot-relative chassis speeds.
   * @param wheelForceFeedforwardsX The robot-relative individual module forces x-component in Newtons.
   * @param wheelForceFeedforwardsY The robot-relative individual module forces y-component in Newtons.
   */
  public void drive(ChassisSpeeds speeds, double[] wheelForceFeedforwardsX, double[] wheelForceFeedforwardsY) {
    _isFieldOriented = false;
    _isOpenLoop = false;
    
    setControl(
        _robotSpeedsRequest.withSpeeds(speeds)
            .withWheelForceFeedforwardsX(wheelForceFeedforwardsX)
            .withWheelForceFeedforwardsY(wheelForceFeedforwardsY));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void startSimThread() {
    _lastSimTime = Utils.getCurrentTimeSeconds();

    // Run simulation at a faster rate so PID gains behave more reasonably
    _simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - _lastSimTime;
      _lastSimTime = currentTime;

      // use the measured time delta, get battery voltage from WPILib
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    
    _simNotifier.startPeriodic(1 / Constants.simUpdateFrequency.in(Hertz));
  }
}
