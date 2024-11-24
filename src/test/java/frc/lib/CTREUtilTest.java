package frc.lib;

import static frc.lib.UnitTestingUtil.reset;
import static frc.lib.UnitTestingUtil.setupTests;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.FaultsTable.Fault;
import frc.lib.FaultsTable.FaultType;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class CTREUtilTest {
  private TalonFX _motor;

  @BeforeEach
  public void setup() {
    setupTests();

    _motor = new TalonFX(1);
  }

  @AfterEach
  public void close() throws Exception {
    reset();

    _motor.close();
  }

  @Test
  public void motorName() {
    var name = CTREUtil.getName(_motor);

    assertEquals(name, "TalonFX (1)");
  }

  @Test
  public void motorAttempt() {
    var name = CTREUtil.getName(_motor);

    var failed = CTREUtil.attempt(() -> StatusCode.ConfigFailed, _motor);

    assert failed;

    FaultLogger.update();

    assert FaultLogger.totalFaults()
        .contains(
            new Fault(
                name + ": Config Apply Failed - " + StatusCode.ConfigFailed.getDescription(),
                FaultType.ERROR));
  }
}
