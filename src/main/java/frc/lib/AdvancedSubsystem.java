package frc.lib;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FaultsTable.Fault;
import frc.lib.FaultsTable.FaultType;
import java.util.HashSet;
import java.util.Set;

public abstract class AdvancedSubsystem extends SubsystemBase
    implements SelfChecked, AutoCloseable {
  // faults and the table containing them
  private Set<Fault> _faults = new HashSet<Fault>();
  private FaultsTable _faultsTable =
      new FaultsTable(
          NetworkTableInstance.getDefault().getTable("Self Check"), getName() + " Faults");

  private boolean _hasError = false;

  public AdvancedSubsystem() {}

  /** Clears this subsystem's faults. */
  protected final void clearFaults() {
    _faults.clear();
    _faultsTable.set(_faults);

    _hasError = false;
  }

  /** Adds a new fault under this subsystem. */
  protected final void addFault(String description, FaultType faultType) {
    _hasError = (faultType == FaultType.ERROR);

    Fault fault = new Fault(description, faultType);

    _faults.add(fault);
    _faultsTable.set(_faults);
  }

  /** Returns the faults belonging to this subsystem. */
  public final Set<Fault> getFaults() {
    return _faults;
  }

  /** Returns whether this subsystem has errors (has fault type of error). */
  public final boolean hasError() {
    return _hasError;
  }

  /** Returns a full Command that self checks this Subsystem for pre-match. */
  public final Command fullSelfCheck() {
    Command selfCheck =
        sequence(runOnce(this::clearFaults), selfCheck(this::addFault).until(this::hasError))
            .withName(getName() + " Self Check");

    return selfCheck;
  }

  @Override
  public void periodic() {
    String currentCommandName = "None";

    if (getCurrentCommand() != null) {
      currentCommandName = getCurrentCommand().getName();
    }

    DogLog.log(getName() + "/Current Command", currentCommandName);
  }
}
