// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest.*;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.FaultLogger;
import frc.lib.FaultsTable;
import frc.lib.FaultsTable.Fault;
import frc.lib.FaultsTable.FaultType;
import frc.lib.InputStream;
import frc.lib.SelfChecked;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.utils.SysId;
import frc.robot.utils.VisionPoseEstimator;
import frc.robot.utils.VisionPoseEstimator.VisionPoseEstimate;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;
import org.photonvision.simulation.VisionSystemSim;

@Logged(strategy = Strategy.OPT_IN)
public class Swerve extends SwerveDrivetrain implements Subsystem, SelfChecked {
  // faults and the table containing them
  private Set<Fault> _faults = new HashSet<Fault>();
  private FaultsTable _faultsTable =
      new FaultsTable(
          NetworkTableInstance.getDefault().getTable("Self Check"), getName() + " Faults");

  private boolean _hasError = false;

  // teleop requests
  private final RobotCentric _robotCentricRequest = new RobotCentric();
  private final FieldCentric _fieldCentricRequest = new FieldCentric();

  private final SwerveDriveBrake _brakeRequest = new SwerveDriveBrake();

  // auton request
  private final ApplyRobotSpeeds _robotSpeedsRequest = new ApplyRobotSpeeds();

  // sysid requests
  private final SysIdSwerveTranslation _translationSysIdRequest = new SysIdSwerveTranslation();
  private final SysIdSwerveSteerGains _steerSysIdRequest = new SysIdSwerveSteerGains();
  private final SysIdSwerveRotation _rotationSysIdRequest =
      new SysIdSwerveRotation(); // how does this work?

  // SysId routine for characterizing translation. This is used to find PID gains for the drive
  // motors.
  private final SysIdRoutine _sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              state -> DogLog.log("SysId Translation Routine State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(_translationSysIdRequest.withVolts(volts)), null, this));

  // SysId routine for characterizing steer. This is used to find PID gains for the steer motors.
  private final SysIdRoutine _sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              state -> DogLog.log("SysId Steer Routine State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(_steerSysIdRequest.withVolts(volts)), null, this));

  // SysId routine for characterizing rotation.
  private final SysIdRoutine _sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              // This is in radians per second², but SysId only supports "volts per second"
              Volts.of(Math.PI / 6).per(Second),
              // This is in radians per second, but SysId only supports "volts"
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              state -> DogLog.log("SysId Rotation Routine State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                // output is actually radians per second, but SysId only supports "volts"
                setControl(_rotationSysIdRequest.withRotationalRate(output.in(Volts)));
                // also log the requested output for SysId
                SignalLogger.writeDouble("Rotational Rate", output.in(Volts));
              },
              null,
              this));

  private double _lastSimTime = 0;
  private Notifier _simNotifier;

  @Logged(name = "Driver Chassis Speeds")
  private final ChassisSpeeds _driverChassisSpeeds = new ChassisSpeeds();

  @Logged(name = "Is Field Oriented")
  private boolean _isFieldOriented = true;

  @Logged(name = "Is Open Loop")
  private boolean _isOpenLoop = true;

  @Logged(name = "Ignore Vision Estimates")
  private boolean _ignoreVisionEstimates = false;

  @Logged(name = VisionConstants.leftArducamName)
  private final VisionPoseEstimator _leftArducam =
      VisionPoseEstimator.buildFromConstants(VisionConstants.leftArducam);

  // for easy iteration with multiple cameras
  private final List<VisionPoseEstimator> _cameras = List.of(_leftArducam);

  private final List<VisionPoseEstimate> _acceptedEstimates = new ArrayList<>();
  private final List<VisionPoseEstimate> _rejectedEstimates = new ArrayList<>();

  private final Set<Pose3d> _detectedTags = new HashSet<>();

  private final VisionSystemSim _visionSystemSim;

  /**
   * Creates a new CommandSwerveDrivetrain.
   *
   * @param drivetrainConstants The CTRE {@link SwerveDrivetrainConstants}. These involve the CAN
   *     Bus name and the Pigeon Id.
   * @param moduleConstants The CTRE {@link SwerveModuleConstants}. The involve constants identical
   *     across all modules (PID constants, wheel radius, etc), and constants unique to each module
   *     (location, device ids, etc).
   */
  public Swerve(
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
          DogLog.log("Swerve/Raw Heading", state.RawHeading);
          DogLog.log("Swerve/Speeds", state.Speeds);
          DogLog.log("Swerve/Desired Speeds", getKinematics().toChassisSpeeds(state.ModuleTargets));
          DogLog.log("Swerve/Module States", state.ModuleStates);
          DogLog.log("Swerve/Desired Module States", state.ModuleTargets);

          double totalDaqs = state.SuccessfulDaqs + state.FailedDaqs;
          totalDaqs = totalDaqs == 0 ? 1 : totalDaqs;

          DogLog.log("Swerve/Odometry Success %", state.SuccessfulDaqs / totalDaqs * 100);
          DogLog.log("Swerve/Odometry Period", state.OdometryPeriod);
        });

    // display all sysid routines
    SysId.displayRoutine("Swerve Translation", _sysIdRoutineTranslation);
    SysId.displayRoutine("Swerve Steer", _sysIdRoutineSteer);
    SysId.displayRoutine("Swerve Rotation", _sysIdRoutineRotation);

    registerFallibles();

    if (Robot.isSimulation()) {
      startSimThread();

      _visionSystemSim = new VisionSystemSim("Vision System Sim");
      _visionSystemSim.addAprilTags(FieldConstants.fieldLayout);

      _cameras.forEach(cam -> _visionSystemSim.addCamera(cam.getCameraSim(), cam.robotToCam));
    } else {
      _visionSystemSim = null;
    }
  }

  // COPIED FROM ADVANCED SUBSYSTEM

  /** Adds a new fault under this subsystem. */
  private final void addFault(String description, FaultType faultType) {
    _hasError = (faultType == FaultType.ERROR);

    Fault fault = new Fault(description, faultType);

    DogLog.logFault(fault.toString());

    _faults.add(fault);
    _faultsTable.set(_faults);
  }

  /** Clears this subsystem's faults. */
  public final void clearFaults() {
    _faults.clear();
    _faultsTable.set(_faults);

    _hasError = false;
  }

  /** Returns the faults belonging to this subsystem. */
  public final Set<Fault> getFaults() {
    return _faults;
  }

  /** Returns whether this subsystem contains the following fault. */
  public final boolean hasFault(String description, FaultType faultType) {
    return _faults.contains(new Fault(description, faultType));
  }

  /** Returns whether this subsystem has errors (has fault type of error). */
  public final boolean hasError() {
    return _hasError;
  }

  /** Returns a full Command that self checks this Subsystem for pre-match. */
  public final Command fullSelfCheck() {
    Command selfCheck =
        sequence(runOnce(this::clearFaults), selfCheck(this::addFault).until(this::hasError))
            .withName(getName() + " Self Check");

    return selfCheck;
  }

  private void registerFallibles() {
    for (SwerveModule module : getModules()) {
      FaultLogger.register(module.getDriveMotor());
      FaultLogger.register(module.getSteerMotor());
      FaultLogger.register(module.getCANcoder());
    }

    FaultLogger.register(getPigeon2());
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

    _simNotifier.setName("Swerve Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simUpdateFrequency.in(Hertz));
  }

  /** Toggles the field oriented boolean. */
  public Command toggleFieldOriented() {
    return brake()
        .withTimeout(0.5)
        .andThen(runOnce(() -> _isFieldOriented = !_isFieldOriented))
        .withName("Toggle Field Oriented");
  }

  /** Brakes the swerve drive (modules form an "X" formation). */
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
        .beforeStarting(
            () -> {
              _isFieldOriented = true;
              _isOpenLoop = false;
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

    if (_isFieldOriented) tempSpeeds.toRobotRelativeSpeeds(getHeading());

    tempStates = getKinematics().toSwerveModuleStates(tempSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(tempStates, SwerveConstants.maxTranslationalSpeed);
    tempSpeeds = getKinematics().toChassisSpeeds(tempStates);

    if (_isFieldOriented) tempSpeeds.toFieldRelativeSpeeds(getHeading());

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

  /** Returns the robot's estimated rotation at the given timestamp. */
  public Rotation2d getHeadingAtTime(double timestamp) {
    return samplePoseAt(timestamp).orElse(getPose()).getRotation();
  }

  /** Wrapper for getting current robot-relative chassis speeds. */
  public ChassisSpeeds getChassisSpeeds() {
    return getState().Speeds;
  }

  // updates pose estimator with vision
  private void updateVisionPoseEstimates() {
    _acceptedEstimates.clear();
    _rejectedEstimates.clear();

    _detectedTags.clear();

    for (VisionPoseEstimator cam : _cameras) {
      cam.update(this::getHeadingAtTime);

      var estimates = cam.getNewEstimates();

      // process estimates
      estimates.forEach(
          (estimate) -> {
            // add all detected tag poses
            for (int id : estimate.detectedTags()) {
              FieldConstants.fieldLayout.getTagPose(id).ifPresent(pose -> _detectedTags.add(pose));
            }

            // add robot poses to their corresponding arrays
            if (estimate.isValid()) _acceptedEstimates.add(estimate);
            else _rejectedEstimates.add(estimate);
          });
    }
  }

  @Override
  public void periodic() {
    // ---- advanced subsystem periodic ----
    String currentCommandName = "None";

    if (getCurrentCommand() != null) {
      currentCommandName = getCurrentCommand().getName();
    }

    DogLog.log(getName() + "/Current Command", currentCommandName);

    // ---- this subsystem's periodic ----
    updateVisionPoseEstimates();

    DogLog.log(
        "Swerve/Accepted Estimates",
        _acceptedEstimates.stream().map(VisionPoseEstimate::pose).toArray(Pose3d[]::new));
    DogLog.log(
        "Swerve/Rejected Estimates",
        _rejectedEstimates.stream().map(VisionPoseEstimate::pose).toArray(Pose3d[]::new));

    DogLog.log("Swerve/Detected Tags", _detectedTags.toArray(Pose3d[]::new));

    if (!_ignoreVisionEstimates) {
      _acceptedEstimates.sort(VisionPoseEstimate.sorter);

      _acceptedEstimates.forEach(
          (e) -> {
            var stdDevs = e.stdDevs();
            addVisionMeasurement(
                e.pose().toPose2d(),
                e.timestamp(),
                VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2]));
          });
    }
  }

  @Override
  public void simulationPeriodic() {
    _visionSystemSim.update(getPose()); // TODO
  }

  private Command selfCheckModule(String name, SwerveModule module) {
    return shiftSequence();
  }

  @Override
  public Command selfCheck(BiConsumer<String, FaultType> faults) {
    return shiftSequence(
        // check all modules individually
        selfCheckModule("Front Left", getModule(0)),
        selfCheckModule("Front Right", getModule(1)),
        selfCheckModule("Back Left", getModule(2)),
        selfCheckModule("Back Right", getModule(3)));
  }

  @Override
  public void close() {
    super.close();

    _simNotifier.close();
  }
}
