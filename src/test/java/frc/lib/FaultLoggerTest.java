package frc.lib;

import static frc.lib.util.UnitTestingUtil.*;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.fault.FaultLogger;
import frc.lib.fault.FaultsTable.Fault;
import frc.lib.fault.FaultsTable.FaultType;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class FaultLoggerTest {
  @BeforeEach
  public void setup() {
    setupTests();
  }

  @AfterEach
  public void close() {
    reset();
  }

  @Test
  public void conditionFaultActivatesAndClears() {
    boolean[] flag = {false};
    FaultLogger.register(() -> flag[0], "cond", FaultType.WARNING);
    var fault = new Fault("cond", FaultType.WARNING);

    flag[0] = true;
    FaultLogger.update();
    assertTrue(FaultLogger.activeFaults().contains(fault));

    flag[0] = false;
    FaultLogger.update();
    assertTrue(FaultLogger.activeFaults().isEmpty());
    assertTrue(FaultLogger.totalFaults().contains(fault));
  }

  @Test
  public void oneShotReportLifecycle() {
    var fault = new Fault("once", FaultType.INFO);

    FaultLogger.report("once", FaultType.INFO);
    FaultLogger.update();
    assertTrue(FaultLogger.activeFaults().contains(fault));

    FaultLogger.update();
    assertFalse(FaultLogger.activeFaults().contains(fault));
    assertTrue(FaultLogger.totalFaults().contains(fault));
  }

  @Test
  public void clearThenRepoll() {
    FaultLogger.register(() -> true, "cond", FaultType.WARNING);
    var fault = new Fault("cond", FaultType.WARNING);

    FaultLogger.update();
    FaultLogger.clear();
    FaultLogger.update();

    assertTrue(FaultLogger.activeFaults().contains(fault));
    assertTrue(FaultLogger.totalFaults().contains(fault));
  }

  @Test
  public void devicesOnTwoBusesPoll() {
    try (var a = new TalonFX(1);
        var b = new TalonFX(2, new CANBus("canivore"))) {
      FaultLogger.register(a);
      FaultLogger.register(b);

      assertDoesNotThrow(
          () -> {
            for (int i = 0; i < 3; i++) FaultLogger.update();
          });
      // sim frames can be stale (not OK), but a bus-grouping bug reports an invalid network
      assertTrue(
          FaultLogger.activeFaults().stream()
              .noneMatch(f -> f.description().contains("network/CAN bus is invalid")));
    }
  }

  @Test
  public void canBusRegisteredWithoutErrorInSim() {
    FaultLogger.register(new CANBus("canivore"));
    FaultLogger.update();
    FaultLogger.update();

    assertTrue(FaultLogger.activeFaults().stream().noneMatch(f -> f.type() == FaultType.ERROR));
  }
}
