package edu.wpi.first.wpilibj2.command;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import javax.annotation.Nullable;

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
  default Command functional(@Nullable Runnable start, @Nullable Runnable run,
      @Nullable Consumer<Boolean> end,
      @Nullable BooleanSupplier isFinished) {
    return new FunctionalCommand(Util.nullFallback(start, Util::doNothing),
        Util.nullFallback(run, Util::doNothing),
        Util.nullFallback(end, Util::justConsume),
        Util.nullFallback(isFinished, Util::alwaysFalse), this);
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
