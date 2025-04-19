package edu.wpi.first.wpilibj2.command;

import java.util.function.Supplier;

// 元のドキュメント: Original document:
// https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Subsystem.html

public interface SubSystemEx extends Subsystem {
  /**
   * Constructs a command that runs an action every iteration until interrupted.
   *
   * @param action - the action to run
   * @see RunCommand
   */
  default Command run(Runnable action) {
    return new RunCommand(action, this);
  }

  /**
   * Constructs a command that runs an action every iteration until interrupted, and then runs a
   * second action.
   *
   * @param run - the action to run every iteration
   * @param end - the action to run on interrupt
   * @see RunEndCommand
   */
  default Command runEnd(Runnable run, Runnable end) {
    return new RunEndCommand(run, end, this);
  }

  /**
   * Constructs a command that runs an action once and finishes.
   *
   * @param action - the action to run
   * @see InstantCommand
   */
  default Command runOnce(Runnable action) {
    return new InstantCommand(action, this);
  }

  /**
   * Constructs a command that runs an action once and another action when the command is
   * interrupted.
   *
   * @param start - the action to run on start
   * @param end - the action to run on interrupt
   * @see StartEndCommand
   */
  default Command startEnd(Runnable start, Runnable end) {
    return new StartEndCommand(start, end, this);
  }

  /**
   * Constructs a command that runs an action once and then runs another action every iteration
   * until interrupted.
   *
   * @param start - the action to run on start
   * @param run - the action to run every iteration
   * @see RunStartCommand
   */
  default Command startRun(Runnable start, Runnable run) {
    return new RunStartCommand(start, run);
  }

  /**
   * Constructs a DeferredCommand with the provided supplier. This subsystem is added as a
   * requirement.
   *
   * @param supplier - the command supplier.
   * @see DeferredCommand
   */
  default Command defer(Supplier<Command> supplier) {
    return new DeferredCommand(supplier, this);
  }
}
