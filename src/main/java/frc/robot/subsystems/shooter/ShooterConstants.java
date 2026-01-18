package frc.robot.subsystems.shooter;

public final class ShooterConstants {
  private ShooterConstants() {}

  // Max/min shooter speeds (rad/s or units your encoders use)
  public static final double MAX_SHOOTER_SPEED = 5700; // Example, adjust
  public static final double MIN_SHOOTER_SPEED = 0;

  // Voltage applied to feeder during firing
  public static final double FEEDER_FIRING_VOLTAGE = 8;

  // Allowable velocity tolerance for "at speed"
  public static final double SHOOTER_VELOCITY_RANGE = 50; // units/sec

  // Shuffleboard defaults
  public static final double DEFAULT_IDLE_SPEED = 4.9;
  public static final double DEFAULT_FIRE_BOOST = 0;
  public static final double DEFAULT_SHOOTER_VEL = -1;
}
