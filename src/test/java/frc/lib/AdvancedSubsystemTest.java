// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import static frc.lib.UnitTestingUtil.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.FaultsTable.FaultType;
import java.util.function.BiConsumer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class AdvancedSubsystemTest {
  static final double DELTA = 1e-2; // acceptable deviation range

  private TestImpl sub = new TestImpl();

  @BeforeEach
  public void setup() {
    setupTests();
  }

  @AfterEach
  public void close() throws Exception {
    reset(sub);
  }

  @Test
  public void subsystemSelfCheck() {
    // TODO
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
        return Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (_fault1) faultAdder.accept("FAULT 1", FaultType.ERROR);
                }),
            Commands.runOnce(
                () -> {
                  if (_fault2) faultAdder.accept("FAULT 2", FaultType.ERROR);
                }),
            Commands.runOnce(
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
      return Commands.sequence(
          _io.selfCheck(faultAdder), // self check io devices first
          runOnce(
              () -> {
                if (speed() < 2) faultAdder.accept("TOO SLOW", FaultType.WARNING);
              }) // then check the whole subsystem
          );
    }
  }
}
