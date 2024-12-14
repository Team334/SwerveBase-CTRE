// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.lib.UnitTestingUtil.*;
import static frc.lib.UnitTestingUtil.run;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FaultsTable.Fault;
import frc.lib.FaultsTable.FaultType;
import java.util.function.BiConsumer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class AdvancedSubsystemTest {
  static final double DELTA = 1e-2; // acceptable deviation range

  private TestImpl _sub;

  @BeforeEach
  public void setup() {
    setupTests();

    _sub = new TestImpl();
  }

  @AfterEach
  public void close() throws Exception {
    reset(_sub);
  }

  @Test
  public void robotIsEnabled() {
    assert DriverStation.isEnabled();
  }

  @Test
  public void addFault() {
    _sub.addFault("FAULT 1", FaultType.ERROR);

    assertEquals(_sub.getFaults().size(), 1, DELTA);

    assert _sub.hasFault("FAULT 1", FaultType.ERROR);
  }

  @Test
  public void faultEquality() {
    Fault f1 = new Fault("FAULT 1", FaultType.ERROR);
    Fault f2 = new Fault("FAULT 1", FaultType.ERROR);

    assert f1.equals(f2);
  }

  @Test
  public void duplicateFaults() {
    _sub.addFault("FAULT 1", FaultType.ERROR);
    _sub.addFault("FAULT 1", FaultType.ERROR);

    assertEquals(_sub.getFaults().size(), 1, DELTA);
  }

  @Test
  public void clearFaults() {
    _sub.addFault("FAULT 1", FaultType.WARNING);
    _sub.addFault("FAULT 2", FaultType.ERROR);

    _sub.clearFaults();

    assertEquals(_sub.getFaults().size(), 0, DELTA);
  }

  @Test
  public void hasError() {
    _sub.addFault("FAULT 1", FaultType.ERROR);

    assert _sub.hasError();
  }

  @Test
  public void selfCheckFinish() {
    runToCompletion(_sub.fullSelfCheck());

    // should give FAULT 3 error
    assertEquals(_sub.getFaults().size(), 3, DELTA);

    // should contain these faults
    assert _sub.hasFault("FAULT 1", FaultType.WARNING);
    assert _sub.hasFault("FAULT 2", FaultType.WARNING);
    assert _sub.hasFault("FAULT 3", FaultType.ERROR);
  }

  @Test
  public void currentCommandName() {
    var test = idle(_sub).withName("Test Command");

    run(test, 3);

    assertEquals("Test Command", _sub.getCurrentCommand().getName()); // TODO
  }

  public class TestImpl extends AdvancedSubsystem {
    private final BaseIO _io = new TestIO();

    private interface BaseIO extends SelfChecked {
      public double getEncoderSpeed();
    }

    // irl this would be for example "TalonIO" (handles real/sim) or "NoneIO" (handles no subsystem)
    private class TestIO implements BaseIO {
      private final boolean _fault1 = true;
      private final boolean _fault2 = true;
      private final boolean _fault3 = true;

      @Override
      public double getEncoderSpeed() {
        return 1.0;
      }

      @Override
      public Command selfCheck(BiConsumer<String, FaultType> faults) {
        return shiftSequence(
            runOnce(
                () -> {
                  if (_fault1) faults.accept("FAULT 1", FaultType.WARNING);
                }),
            runOnce(
                () -> {
                  if (_fault2) faults.accept("FAULT 2", FaultType.WARNING);
                }),
            runOnce(
                () -> {
                  if (_fault3) faults.accept("FAULT 3", FaultType.ERROR);
                }));
      }
    }

    // uses the io
    public double speed() {
      return _io.getEncoderSpeed();
    }

    @Override
    public Command selfCheck(BiConsumer<String, FaultType> faults) {
      return shiftSequence(
          _io.selfCheck(faults), // self check io devices first
          runOnce(
              () -> {
                if (speed() < 2) faults.accept("TOO SLOW", FaultType.WARNING);
              }) // then check the whole subsystem
          );
    }

    @Override
    public void close() throws Exception {}
  }
}
