// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AdvancedSubsystem;
import frc.lib.FaultsTable.FaultType;
import frc.lib.SelfChecked;
import java.util.function.BiConsumer;

/** Advanced Subsystem Test (self check test) */
public class AdvancedTest extends AdvancedSubsystem {
  private final SomeIO _io = new SomeIO();

  // represents IO, which can either be talonfx/rev or none
  private class SomeIO implements SelfChecked {
    private final boolean _fault1 = true;
    private final boolean _fault2 = true;
    private final boolean _fault3 = false;

    public double readSomeSensor() {
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

  public double speed() {
    return _io.readSomeSensor();
  }

  @Override
  public Command selfCheck(BiConsumer<String, FaultType> faultAdder) {
    // return Commands.sequence(
    //     _io.selfCheck(faultAdder), // self check io devices first
    //     runOnce(() -> { if (speed() != 2) faultAdder.accept("TOO SLOW", FaultType.WARNING); }) //
    // then check the whole subsystem
    // );
    return _io.selfCheck(faultAdder); // wpilib bug present here, fault 2 shouldn't run but it does
  }
}
