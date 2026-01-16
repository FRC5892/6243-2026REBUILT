package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.LoggedTunableNumber;

/**
 * All shooter tuning parameters centralized.
 * LoggedTunableNumbers allow live tuning via AdvantageKit.
 */
public final class ShooterConstants {

    private ShooterConstants() {}

    /** PIDF gains for main shooter velocity */
    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 0.001);
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI", 0.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0.0001);
    public static final LoggedTunableNumber kFF = new LoggedTunableNumber("Shooter/kFF", 0.0003398478);

    /** Voltage caps */
    public static final LoggedTunableNumber kIdleVoltage = new LoggedTunableNumber("Shooter/IdleVoltage", 4.9);      // SPEED LIMIT
    public static final LoggedTunableNumber kFeederVoltage = new LoggedTunableNumber("Shooter/FeederVoltage", 8.0);  // SPEED LIMIT
    public static final LoggedTunableNumber kFiringBoost = new LoggedTunableNumber("Shooter/FiringBoost", 1.0);      // SPEED LIMIT

    /** Velocity tolerance for "at speed" check */
    public static final LoggedTunableNumber kVelocityTolerance = new LoggedTunableNumber("Shooter/VelocityTolerance", 50.0); // RPM
}
