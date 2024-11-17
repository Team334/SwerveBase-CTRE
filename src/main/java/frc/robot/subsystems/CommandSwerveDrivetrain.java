// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest.*;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.lib.InputStream;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

@Logged(strategy = Strategy.OPT_IN)
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  // teleop requests
  private final RobotCentric _robotCentricRequest = new RobotCentric();
  private final FieldCentric _fieldCentricRequest = new FieldCentric();

  private final SwerveDriveBrake _brakeRequest = new SwerveDriveBrake();

  // auton request
  private final ApplyRobotSpeeds _robotSpeedsRequest = new ApplyRobotSpeeds();

  private double _lastSimTime = 0;
  private Notifier _simNotifier;

  @Logged(name = "Driver Chassis Speeds")
  private final ChassisSpeeds _driverChassisSpeeds = new ChassisSpeeds();

  @Logged(name = "Is Field Oriented")
  private boolean _isFieldOriented = true;

  @Logged(name = "Is Open Loop")
  private boolean _isOpenLoop = true;

  /**
   * Creates a new CommandSwerveDrivetrain.
   *
   * @param drivetrainConstants The CTRE {@link SwerveDrivetrainConstants}. These involve the CAN
   *     Bus name and the Pigeon Id.
   * @param moduleConstants The CTRE {@link SwerveModuleConstants}. The involve constants identical
   *     across all modules (PID constants, wheel radius, etc), and constants unique to each module
   *     (location, device ids, etc).
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... moduleConstants) {
    super(drivetrainConstants, SwerveConstants.odometryFrequency.in(Hertz), moduleConstants);

    _robotCentricRequest
        .withDeadband(SwerveConstants.translationalDeadband)
        .withRotationalDeadband(SwerveConstants.rotationalDeadband);

    _fieldCentricRequest
        .withDeadband(SwerveConstants.translationalDeadband)
        .withRotationalDeadband(SwerveConstants.rotationalDeadband);

    _robotSpeedsRequest.withDriveRequestType(DriveRequestType.Velocity);

    registerTelemetry(
        state -> {
          DogLog.log("Swerve/Pose", state.Pose);
          DogLog.log("Swerve/Speeds", state.Speeds);
          DogLog.log("Swerve/Desired Speeds", getKinematics().toChassisSpeeds(state.ModuleTargets));
          DogLog.log("Swerve/Module States", state.ModuleStates);
          DogLog.log("Swerve/Desired Module States", state.ModuleTargets);

          double totalDaqs = state.SuccessfulDaqs + state.FailedDaqs;
          totalDaqs = totalDaqs == 0 ? 1 : totalDaqs;

          DogLog.log("Swerve/Odometry Success %", state.SuccessfulDaqs / totalDaqs * 100);
          DogLog.log("Swerve/Odometry Period", state.OdometryPeriod);
        });

    // TODO: idk if this will conflict with the actual scheduled commands
    RobotModeTriggers.autonomous()
        .onTrue(
            runOnce(
                () -> {
                  _isFieldOriented = false;
                  _isOpenLoop = false;
                }));
    RobotModeTriggers.teleop()
        .onTrue(
            runOnce(
                () -> {
                  _isFieldOriented = true;
                  _isOpenLoop = true;
                }));

    if (RobotBase.isSimulation()) startSimThread();
  }

  /** Toggles the field oriented boolean. */
  public Command toggleFieldOriented() {
    return brake()
        .withTimeout(0.5)
        .andThen(runOnce(() -> _isFieldOriented = !_isFieldOriented))
        .withName("Toggle Field Oriented");
  }

  /** Brakes the swerve drive (module form an "X" formation). */
  public Command brake() {
    return run(() -> setControl(_brakeRequest)).withName("Brake");
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
          drive(velX.get(), velY.get(), velOmega.get());
        })
        .withName("Drive");
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

    // go through a couple of steps to ensure that input speeds are actually achievable
    ChassisSpeeds tempSpeeds = _driverChassisSpeeds;
    SwerveModuleState[] tempStates;

    if (_isFieldOriented)
      tempSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(tempSpeeds, getHeading());

    tempStates = getKinematics().toSwerveModuleStates(tempSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(tempStates, SwerveConstants.maxTranslationalSpeed);
    tempSpeeds = getKinematics().toChassisSpeeds(tempStates);

    if (_isFieldOriented)
      tempSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(tempSpeeds, getHeading());

    velX = tempSpeeds.vxMetersPerSecond;
    velY = tempSpeeds.vyMetersPerSecond;
    velOmega = tempSpeeds.omegaRadiansPerSecond;

    if (_isFieldOriented) {
      setControl(
          _fieldCentricRequest
              .withVelocityX(velX)
              .withVelocityY(velY)
              .withRotationalRate(velOmega)
              .withDriveRequestType(
                  _isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity));
    } else {
      setControl(
          _robotCentricRequest
              .withVelocityX(velX)
              .withVelocityY(velY)
              .withRotationalRate(velOmega)
              .withDriveRequestType(
                  _isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity));
    }
  }

  /**
   * Drives the swerve drive. This configuration is robot oriented and closed-loop, specifically
   * meant for auton.
   *
   * @param speeds The robot-relative chassis speeds.
   * @param wheelForceFeedforwardsX The robot-relative individual module forces x-component in
   *     Newtons.
   * @param wheelForceFeedforwardsY The robot-relative individual module forces y-component in
   *     Newtons.
   */
  public void drive(
      ChassisSpeeds speeds, double[] wheelForceFeedforwardsX, double[] wheelForceFeedforwardsY) {
    setControl(
        _robotSpeedsRequest
            .withSpeeds(speeds)
            .withWheelForceFeedforwardsX(wheelForceFeedforwardsX)
            .withWheelForceFeedforwardsY(wheelForceFeedforwardsY));
  }

  /** Wrapper for getting estimated pose. */
  public Pose2d getPose() {
    return getState().Pose;
  }

  /** Wrapper for getting estimated heading. */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /** Wrapper for getting current robot-relative chassis speeds. */
  public ChassisSpeeds getChassisSpeeds() {
    return getState().Speeds;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void startSimThread() {
    _lastSimTime = Utils.getCurrentTimeSeconds();

    // Run simulation at a faster rate so PID gains behave more reasonably
    _simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - _lastSimTime;
              _lastSimTime = currentTime;

              // use the measured time delta, get battery voltage from WPILib
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });

    _simNotifier.startPeriodic(1 / Constants.simUpdateFrequency.in(Hertz));
  }
}
