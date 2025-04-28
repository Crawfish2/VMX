package edu.wpi.first.wpilibj2.command;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import javax.annotation.Nullable;

public class Commands {
  /**
   * Constructs a command that runs an action every iteration until interrupted.
   *
   * @param action - the action to run
   * @see RunCommand
   */
  public static CommandBase run(Runnable action, Subsystem... requirements) {
    return new RunCommand(action, requirements);
  }

  /**
   * Constructs a command that runs an action every iteration until interrupted, and then runs a
   * second action.
   *
   * @param run - the action to run every iteration
   * @param end - the action to run on interrupt
   * @see RunEndCommand
   */
  public static CommandBase runEnd(Runnable run, Runnable end, Subsystem... requirements) {
    return new RunEndCommand(run, end, requirements);
  }

  /**
   * Constructs a command that runs an action once and finishes.
   *
   * @param action - the action to run
   * @see InstantCommand
   */
  public static CommandBase runOnce(Runnable action, Subsystem... requirements) {
    return new InstantCommand(action, requirements);
  }

  /**
   * Constructs a command that runs an action once and another action when the command is
   * interrupted.
   *
   * @param start - the action to run on start
   * @param end - the action to run on interrupt
   * @see StartEndCommand
   */
  public static CommandBase startEnd(Runnable start, Runnable end, Subsystem... requirements) {
    return new StartEndCommand(start, end, requirements);
  }

  /**
   * Constructs a command that runs an action once and then runs another action every iteration
   * until interrupted.
   *
   * @param start - the action to run on start
   * @param run - the action to run every iteration
   * @see RunStartCommand
   */
  public static CommandBase startRun(Runnable start, Runnable run, Subsystem... requirements) {
    return new RunStartCommand(start, run);
  }

  /**
   * Constructs a DeferredCommand with the provided supplier. This subsystem is added as a
   * requirement.
   *
   * @param supplier - the command supplier.
   * @see DeferredCommand
   */
  public static CommandBase defer(Supplier<Command> supplier, Subsystem... requirements) {
    return new DeferredCommand(supplier, requirements);
  }

  // 追加のメソッド

  /**
   * 開始時、定期実行時、終了時、終了条件を元にコマンドを作成する
   * 何も実行したくないところは、nullを入れてもよい
   *
   * @param start - 開始時のアクション
   * @param run - 実行中のアクション
   * @param end - 終了時のアクション
   * @param isFinished - 終了条件
   * @see FunctionalCommand
   */
  public static CommandBase functional(@Nullable Runnable start, @Nullable Runnable run,
      @Nullable Consumer<Boolean> end,
      @Nullable BooleanSupplier isFinished, Subsystem... requirements) {
    return new FunctionalCommand(Util.nullFallback(start, Util::doNothing),
        Util.nullFallback(run, Util::doNothing),
        Util.nullFallback(end, Util::justConsume),
        Util.nullFallback(isFinished, Util::alwaysFalse), requirements);
  }


  public static CommandBase runDeadline(Runnable run, BooleanSupplier isFinished,
      Subsystem... requirements) {
    return new RunDeadlineCommand(run, isFinished, requirements);
  }

}



class Util {
  static void doNothing() {
    return;
  }

  static void justConsume(Object value) {
    return;
  }

  static boolean alwaysFalse() {
    return false;
  }

  static <T> T nullFallback(@Nullable T value, T fallback) {
    return Objects.nonNull(value) ? value : fallback;
  }
}
