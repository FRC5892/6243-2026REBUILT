package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem logic.
 * AdvantageKit logging and replay-ready.
 * Uses ShooterIO abstraction for real / sim hardware.
 */
public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    /** Set shooter closed-loop velocity */
    public void setShooterSpeed(double velocityRPM, boolean firingBoost) {
        if (Math.abs(velocityRPM) < 1.0) {
            io.setShooterVoltage(0.0); // SPEED LIMIT: deadband
            return;
        }
        double ffVolts = firingBoost ? ShooterConstants.kFiringBoost.get() : 0.0; // SPEED LIMIT
        io.setShooterVelocity(velocityRPM, ffVolts);
    }

    /** Idle shooter spin */
    public void setIdle() {
        io.setShooterVoltage(ShooterConstants.kIdleVoltage.get()); // SPEED LIMIT
        setFiring(false);
    }

    /** Fire feeder */
    public void setFiring(boolean firing) {
        io.setFeederVoltage(firing ? ShooterConstants.kFeederVoltage.get() : 0.0); // SPEED LIMIT
    }

    /** Check if shooter is at target RPM */
    public boolean isAtSpeed(double targetRPM) {
        return Math.abs(inputs.mainVelocityRPM - targetRPM) < ShooterConstants.kVelocityTolerance.get(); // SPEED LIMIT
    }

    /** Characterization hook for voltage/velocity sweeps */
    public void runCharacterization(double testVoltage) {
        io.setShooterVoltage(testVoltage);
        // AdvantageKit logs inputs automatically
    }

    public double getShooterVelocity() {
        return inputs.mainVelocityRPM;
    }

    public double getFeederCurrent() {
        return inputs.feederCurrentAmps;
    }
}
