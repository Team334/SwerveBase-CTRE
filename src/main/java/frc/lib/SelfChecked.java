package frc.lib;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.FaultsTable.FaultType;
import java.util.function.BiConsumer;

public interface SelfChecked {
  /**
   * Returns a Command that self checks this system.
   *
   * @param faults A consumer that adds a given fault to the subsystem with the specified fault
   *     type.
   */
  public default Command selfCheck(BiConsumer<String, FaultType> faults) {
    return none();
  }

  /**
   * Places an empty instant command at the front of a sequential command group.
   *
   * <p>This is useful because the second self check command gets called in the same iteration as
   * the first one (due to the nature of sequential command groups in wpilib).
   *
   * @param commands The commands to include in this composition.
   */
  public default Command shiftSequence(Command... commands) {
    var shiftedSequence = new SequentialCommandGroup(none());

    shiftedSequence.addCommands(commands);

    return shiftedSequence;
  }
}
