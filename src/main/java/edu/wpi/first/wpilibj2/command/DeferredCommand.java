package edu.wpi.first.wpilibj2.command;

import java.util.function.Supplier;

// 元のドキュメント: Original document:
// https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/DeferredCommand.html

/**
 * Defers Command construction to runtime. Runs the command returned by a supplier when this command
 * is initialized, and ends when it ends. Useful for performing runtime tasks before creating a new
 * command. If this command is interrupted, it will cancel the command.
 * Note that the supplier must create a new Command each call. For selecting one of a preallocated
 * set of commands, use SelectCommand.
 */
public class DeferredCommand extends CommandBase {
  private final Supplier<Command> supplier;
  private Command command;

  /*
   * Creates a new DeferredCommand that directly runs the supplied command when initialized, and
   * ends when it ends.
   */
  public DeferredCommand(Supplier<Command> supplier, Subsystem... requirements) {
    this.supplier = supplier;
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    command = supplier.get();
    command.initialize();
  }

  @Override
  public void execute() {
    command.execute();
  }

  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return command.isFinished();
  }
}
