// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.SignalLogger;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.FaultLogger;
import frc.lib.InputStream;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged(strategy = Strategy.OPT_IN)
public class Robot extends TimedRobot {
  // controllers
  private final CommandXboxController _driverController =
      new CommandXboxController(Ports.driverController);

  // subsystems
  @Logged(name = "Swerve")
  private Swerve _swerve = TunerConstants.createDrivetrain();

  private Command _autonomousCommand = Autos.none();

  private boolean _fileOnlySet = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // set up loggers
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
    Epilogue.bind(this);
    SignalLogger.start();

    setFileOnly(false); // file-only once connected to fms

    DriverStation.silenceJoystickConnectionWarning(isSimulation());

    FaultLogger.setup();

    configureBindings();

    SmartDashboard.putData(
        "Robot Self Check",
        sequence(
                runOnce(() -> DataLogManager.log("Robot Self Check Started!")),
                _swerve.fullSelfCheck(),
                runOnce(() -> DataLogManager.log("Robot Self Check Successful!")))
            .withName("Robot Self Check"));

    addPeriodic(FaultLogger::update, 1);
  }

  // set logging to be file only or not
  private void setFileOnly(boolean fileOnly) {
    DogLog.setOptions(DogLog.getOptions().withNtPublish(!fileOnly));

    Epilogue.configure(
        config -> {
          // TODO
        });
  }

  private void configureBindings() {
    _swerve.setDefaultCommand(
        _swerve.drive(
            InputStream.of(_driverController::getLeftY)
                .negate()
                .scale(SwerveConstants.maxTranslationalSpeed.in(MetersPerSecond)),
            InputStream.of(_driverController::getLeftX)
                .negate()
                .scale(SwerveConstants.maxTranslationalSpeed.in(MetersPerSecond)),
            InputStream.of(_driverController::getRightX)
                .negate()
                .scale(SwerveConstants.maxAngularSpeed.in(RadiansPerSecond))));

    _driverController.x().whileTrue(_swerve.brake());
    _driverController.a().onTrue(_swerve.toggleFieldOriented());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if (DriverStation.isFMSAttached() && !_fileOnlySet) {
      setFileOnly(true);

      _fileOnlySet = true;
    }
  }

  /** This autonomous runs the autonomous command. */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    if (_autonomousCommand != null) {
      _autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (_autonomousCommand != null) {
      _autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
}
