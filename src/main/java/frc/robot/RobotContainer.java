// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.InputStream;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged(strategy = Strategy.OPT_IN)
public class RobotContainer {
	// controllers
  private final CommandXboxController _driverController = new CommandXboxController(Ports.driverController);

  // subsystems
  @Logged(name = "Swerve")
  private CommandSwerveDrivetrain _swerve = TunerConstants.createDrivetrain();

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		_swerve.setDefaultCommand(_swerve.drive(
				InputStream.of(_driverController::getLeftY)
						.negate()
						.scale(SwerveConstants.maxTranslationSpeed.in(MetersPerSecond)),
				InputStream.of(_driverController::getLeftX)
						.negate()
						.scale(SwerveConstants.maxTranslationSpeed.in(MetersPerSecond)),
				InputStream.of(_driverController::getRightX)
						.negate()
						.scale(SwerveConstants.maxAngularSpeed.in(RadiansPerSecond))));

		configureDriverController();
	}

	private void configureDriverController() {}

  /**
   * @return The command to run in autonomous.
   */
	public Command getAutonomousCommand() {
		return Autos.none();
	}
} 
