// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.kinetics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import java.util.Objects;

/** Represents the wheel positions for a Killough drive drivetrain. */
public class KilloughDriveWheelPositions
    implements Interpolatable<KilloughDriveWheelPositions> {
  /** Distance measured by the forward left wheel. */
  public double forwardLeftMeters;

  /** Distance measured by the forward right wheel. */
  public double forwardRightMeters;

  /** Distance measured by the back wheel. */
  public double backMeters;

  /** Constructs a KilloughDriveWheelPositions with zeros for all member fields. */
  public KilloughDriveWheelPositions() {}

  /**
   * Constructs a KilloughDriveWheelPositions.
   *
   * @param forwardLeftMeters Distance measured by the forward left wheel.
   * @param forwardRightMeters Distance measured by the forward right wheel.
   * @param backMeters Distance measured by the back wheel.
   */
  public KilloughDriveWheelPositions(
      double forwardLeftMeters, double forwardRightMeters, double backMeters) {
    this.forwardLeftMeters = forwardLeftMeters;
    this.forwardRightMeters = forwardRightMeters;
    this.backMeters = backMeters;
  }

  @Override
  public boolean equals(Object obj) {
    if (!(obj instanceof KilloughDriveWheelPositions)) {
      return false;
    }
    KilloughDriveWheelPositions other = (KilloughDriveWheelPositions) obj;
    return Math.abs(other.forwardLeftMeters - forwardLeftMeters) < 1E-9
        && Math.abs(other.forwardRightMeters - forwardRightMeters) < 1E-9
        && Math.abs(other.backMeters - backMeters) < 1E-9;
  }

  @Override
  public int hashCode() {
    return Objects.hash(forwardLeftMeters, forwardRightMeters, backMeters);
  }

  @Override
  public String toString() {
    return String.format(
        "KilloughDriveWheelPositions(Forward Left: %.2f m, Forward Right: %.2f m, "
            + "Back: %.2f m)",
        forwardLeftMeters, forwardRightMeters, backMeters);
  }

  @Override
  public KilloughDriveWheelPositions interpolate(KilloughDriveWheelPositions endValue, double t) {
    return new KilloughDriveWheelPositions(
        MathUtil.interpolate(this.forwardLeftMeters, endValue.forwardLeftMeters, t),
        MathUtil.interpolate(this.forwardRightMeters, endValue.forwardRightMeters, t),
        MathUtil.interpolate(this.backMeters, endValue.backMeters, t));
  }
}
