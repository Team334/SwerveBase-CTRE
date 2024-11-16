// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.InputStream;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged
public class Robot extends TimedRobot {
  // controllers
  private final CommandXboxController _driverController =
      new CommandXboxController(Ports.driverController);

  // subsystems
  @Logged(name = "Swerve")
  private CommandSwerveDrivetrain _swerve = TunerConstants.createDrivetrain();

  @NotLogged private Command _autonomousCommand = Autos.none();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // set up loggers
    DogLog.setOptions(new DogLogOptions().withNtPublish(true));
    Epilogue.bind(this);
    SignalLogger.start();

    DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());

    _swerve.setDefaultCommand(
        _swerve.drive(
            InputStream.of(_driverController::getLeftY)
                .negate()
                .scale(SwerveConstants.maxTranslationSpeed.in(MetersPerSecond)),
            InputStream.of(_driverController::getLeftX)
                .negate()
                .scale(SwerveConstants.maxTranslationSpeed.in(MetersPerSecond)),
            InputStream.of(_driverController::getRightX)
                .negate()
                .scale(SwerveConstants.maxAngularSpeed.in(RadiansPerSecond))));
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command. */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    if (_autonomousCommand != null) {
      _autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

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

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
