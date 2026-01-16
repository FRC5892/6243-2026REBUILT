package frc.robot.subsystems.shooter;

/**
 * Hardware abstraction for Shooter subsystem.
 * Supports real hardware, simulation, and AdvantageKit logging.
 */
public interface ShooterIO {

    class ShooterIOInputs {
        public double mainVelocityRPM;
        public double followerVelocityRPM;
        public double feederVelocityRPM;
        public double mainCurrentAmps;
        public double followerCurrentAmps;
        public double feederCurrentAmps;
        public double appliedOutput;
        public boolean isFiring;
    }

    /** Update sensor readings */
    void updateInputs(ShooterIOInputs inputs);

    /** Closed-loop shooter velocity */
    void setShooterVelocity(double velocityRPM, double ffVolts);

    /** Open-loop voltage control for shooter */
    void setShooterVoltage(double volts);

    /** Open-loop voltage control for feeder */
    void setFeederVoltage(double volts);
}
