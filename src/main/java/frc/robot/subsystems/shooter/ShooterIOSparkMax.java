package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

public class ShooterIOSparkMax implements ShooterIO {

    private final CANSparkMax shooterMotor;
    private final CANSparkMax followerMotor;
    private final CANSparkMax feederMotor;

    private final RelativeEncoder shooterEncoder;
    private final SparkMaxPIDController shooterPID;

    public ShooterIOSparkMax() {

        // Initialize motors
        shooterMotor = new CANSparkMax(17, CANSparkMaxLowLevel.MotorType.kBrushless);
        followerMotor = new CANSparkMax(37, CANSparkMaxLowLevel.MotorType.kBrushless);
        feederMotor   = new CANSparkMax(58, CANSparkMaxLowLevel.MotorType.kBrushless);

        // Shooter setup
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setSmartCurrentLimit(40);                  // SPEED LIMIT
        shooterMotor.enableVoltageCompensation(11.0);          // SPEED LIMIT
        shooterMotor.setIdleMode(IdleMode.kCoast);             // SPEED LIMIT

        shooterEncoder = shooterMotor.getEncoder();
        shooterEncoder.setVelocityConversion(0.5);             // SPEED LIMIT
        shooterEncoder.setPositionConversion(0.5);             // SPEED LIMIT

        shooterPID = shooterMotor.getPIDController();
        shooterPID.setP(ShooterConstants.kP.get());
        shooterPID.setI(ShooterConstants.kI.get());
        shooterPID.setD(ShooterConstants.kD.get());
        shooterPID.setFF(ShooterConstants.kFF.get());

        // Follower setup
        followerMotor.restoreFactoryDefaults();
        followerMotor.follow(shooterMotor, true);
        followerMotor.setSmartCurrentLimit(40);                // SPEED LIMIT
        followerMotor.enableVoltageCompensation(12.0);         // SPEED LIMIT
        followerMotor.setIdleMode(IdleMode.kCoast);            // SPEED LIMIT

        // Feeder setup
        feederMotor.restoreFactoryDefaults();
        feederMotor.setSmartCurrentLimit(30);                  // SPEED LIMIT
        feederMotor.enableVoltageCompensation(12.0);          // SPEED LIMIT
        feederMotor.setIdleMode(IdleMode.kBrake);              // SPEED LIMIT
        feederMotor.setInverted(true);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.mainVelocityRPM = shooterEncoder.getVelocity();
        inputs.followerVelocityRPM = followerMotor.getEncoder().getVelocity();
        inputs.feederVelocityRPM = feederMotor.getEncoder().getVelocity();

        inputs.mainCurrentAmps = shooterMotor.getOutputCurrent();
        inputs.followerCurrentAmps = followerMotor.getOutputCurrent();
        inputs.feederCurrentAmps = feederMotor.getOutputCurrent();

        inputs.appliedOutput = shooterMotor.getAppliedOutput();
        inputs.isFiring = feederMotor.getAppliedOutput() > 0.0;
    }

    @Override
    public void setShooterVelocity(double velocityRPM, double ffVolts) {
        shooterPID.setReference(velocityRPM, CANSparkMax.ControlType.kVelocity, 0, ffVolts); // SPEED LIMIT
    }

    @Override
    public void setShooterVoltage(double volts) {
        shooterMotor.setVoltage(volts); // SPEED LIMIT
    }

    @Override
    public void setFeederVoltage(double volts) {
        feederMotor.setVoltage(volts); // SPEED LIMIT
    }
}
