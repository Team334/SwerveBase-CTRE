// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.lib.UnitTestingUtil.*;

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
  public void subsystemSelfCheck() {
    runToCompletion(_sub.fullSelfCheck());

    // should give FAULT 1 error
    assert _sub.hasError();

    // should only give FAULT 1 error and stop there
    assert _sub.getFaults().contains(new Fault("FAULT 1", FaultType.ERROR));
    assert !_sub.getFaults().contains(new Fault("FAULT 2", FaultType.ERROR));
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
      private final boolean _fault3 = false;

      @Override
      public double getEncoderSpeed() {
        return 1.0;
      }

      @Override
      public Command selfCheck(BiConsumer<String, FaultType> faultAdder) {
        return sequence(
            runOnce(
                () -> {
                  if (_fault1) faultAdder.accept("FAULT 1", FaultType.ERROR);
                }),
            runOnce(
                () -> {
                  if (_fault2) faultAdder.accept("FAULT 2", FaultType.ERROR);
                }),
            runOnce(
                () -> {
                  if (_fault3) faultAdder.accept("FAULT 3", FaultType.WARNING);
                }));
      }
    }

    // uses the io
    public double speed() {
      return _io.getEncoderSpeed();
    }

    @Override
    public Command selfCheck(BiConsumer<String, FaultType> faultAdder) {
      return sequence(
          _io.selfCheck(faultAdder), // self check io devices first
          runOnce(
              () -> {
                if (speed() < 2) faultAdder.accept("TOO SLOW", FaultType.WARNING);
              }) // then check the whole subsystem
          );
    }

    @Override
    public void close() throws Exception {}
  }
}
