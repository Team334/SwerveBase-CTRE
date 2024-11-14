// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*; 

import edu.wpi.first.wpilibj2.command.Command;

public final class Autos {
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * An auto that doesn't do anything for 15 sec.
   */
  public static Command none() {
    return idle();
  }
}
