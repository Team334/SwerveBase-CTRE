package frc.lib;

import java.util.HashSet;
import java.util.Set;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.lib.FaultsTable.Fault;
import frc.lib.FaultsTable.FaultType;

public class AdvancedSubsystem extends SubsystemBase implements SelfChecked {
  // faults and the table containing them
  private Set<Fault> _faults = new HashSet<Fault>();
  private FaultsTable _faultsTable = new FaultsTable(NetworkTableInstance.getDefault().getTable("Self Check"), getName() + " Faults");

  private boolean _hasError = false;

  public AdvancedSubsystem() {
    RobotModeTriggers.test().onFalse(runOnce(this::clearFaults));
  }

  /** Clears this subsystem's faults. */
  protected final void clearFaults() {
    _faults.clear();
    _faultsTable.set(_faults);

    _hasError = false;
  }

  /** Adds a new fault under this subsystem. */
  protected final void addFault(String description, FaultType faultType) {
    Fault fault = new Fault(description, faultType);

    _faults.add(fault);
    _faultsTable.set(_faults);

    _hasError = faultType == FaultType.ERROR;
  }

  /** Returns whether this subsystem has errors (has fault type of error). */
  public final boolean hasError() {
    return _hasError;
  }

  /** Returns a full Command that self checks this Subsystem for pre-match. */
  public final Command fullSelfCheck() {    
    Command selfCheck = Commands.sequence(
      runOnce(this::clearFaults), // clear all faults and hasError (also adds this subsystem as a requirement)
      selfCheck(this::addFault).until(this::hasError) // self check this subsystem
    ).withName(getName() + " Self Check");

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
