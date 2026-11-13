package frc.robot;

import static frc.lib.util.UnitTestingUtil.*;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SwerveDefenseModeTest {
  private Swerve _swerve;

  @BeforeEach
  public void setup() {
    setupTests();
    _swerve = TunerConstants.createDrivetrain();
  }

  @AfterEach
  public void close() {
    reset(_swerve);
  }

  private CurrentLimitsConfigs rearDriveLimits() {
    var configs = new CurrentLimitsConfigs();
    _swerve.getModule(2).getDriveMotor().getConfigurator().refresh(configs);
    return configs;
  }

  @Test
  public void defenseModeSwapsDriveLimits() {
    _swerve.setDefenseMode(true);
    var on = rearDriveLimits();
    assertEquals(120, on.StatorCurrentLimit, 1e-6);
    assertEquals(60, on.SupplyCurrentLimit, 1e-6);
    assertEquals(4.5, _swerve.getDriverTranslationalVelocity(), 1e-6);

    _swerve.setDefenseMode(false);
    var off = rearDriveLimits();
    assertEquals(80, off.StatorCurrentLimit, 1e-6);
    assertEquals(40, off.SupplyCurrentLimit, 1e-6);
    assertEquals(4.0, _swerve.getDriverTranslationalVelocity(), 1e-6);
  }
}
