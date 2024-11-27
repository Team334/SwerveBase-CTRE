package frc.robot;

import static frc.lib.UnitTestingUtil.reset;
import static frc.lib.UnitTestingUtil.setupTests;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

public class SwerveTest {
  static final double DELTA = 1e-2; // acceptable deviation range

  private Swerve _swerve;

  @BeforeEach
  public void setup() {
    setupTests();

    _swerve = TunerConstants.createDrivetrain();
  }

  @AfterEach
  public void close() throws Exception {
    reset(_swerve);
  }
}
