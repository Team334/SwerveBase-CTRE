package frc.lib;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FaultsTable.FaultType;
import java.util.function.BiConsumer;

public interface SelfChecked {
  /**
   * Returns a Command that self checks this system.
   *
   * @param faultAdder A consumer that adds a given fault to the subsystem with the specified fault
   *     type.
   */
  public default Command selfCheck(BiConsumer<String, FaultType> faultAdder) {
    return none();
  }
}
