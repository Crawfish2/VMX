
package frc.robot;

public final class Constants {

  public static class TitanConstants {
    public static final int TITAN_ID = 42;

    public static final int MOTOR0 = 0;
    public static final int MOTOR1 = 1;
    public static final int MOTOR2 = 2;
    public static final int MOTOR3 = 3;

    /** Encoder pulse per revolution */
    public static final double pulsePerRevolution = 1464;

    public static class DriveConstants {
      public static final int WHEEL_LEFT = MOTOR2;
      public static final int WHEEL_RIGHT = MOTOR0;
      public static final int WHEEL_BACK = MOTOR1;

      public static final int WHEEL_LEFT_ANGLE = 120;
      public static final int WHEEL_RIGHT_ANGLE = -120;
      public static final int WHEEL_BACK_ANGLE = 0;

      /** Omni wheel radius(mm) */
      public static final double wheelRadius = 50;
      /** Gear Ratio between encoder and wheel */
      public static final double gearRatio = 1 / 1;
      /** Pulse per revolution of wheel */
      public static final double wheelPulseRatio = pulsePerRevolution * gearRatio;
      /** Distance per tick */
      public static final double WHEEL_DIST_PER_TICK =
          (Math.PI * 2 * wheelRadius) / wheelPulseRatio;
    }

    public static class ElevatorConstants {
      public static final int ELEVATOR = MOTOR3;

      /** gear radius(mm) */
      public static final double gearRadius = 6; // 3?
      /** Gear Ratio between encoder and wheel */
      public static final double gearRatio = 24.0 / 64;
      /** Pulse per revolution of wheel */
      public static final double wheelPulseRatio = pulsePerRevolution * gearRatio;
      /** Distance per tick */
      public static final double WHEEL_DIST_PER_TICK =
          (Math.PI * 2 * gearRadius) / wheelPulseRatio;
    }
  }

  public static class Ultrasonic {
    public static final int PingPin = 8;
    public static final int EchoPin = 9;
  }


  /** 実行環境がVMX-Pi上かどうか */
  public static final boolean isReal =
      // VMX-PiではHALUtil.getHALRuntimeType()==1となり、
      // RobotBase.isReal()は正しい結果を返さない

      // OSのCPUアーキテクチャの情報を読んで、arm(VMX-Pi)かどうかを判定する
      // armのWindowsなどでは正しく動かないが、暫定的な対応
      System.getProperty("os.arch").contains("arm");

  public static final boolean isSimulation = !isReal;

}
