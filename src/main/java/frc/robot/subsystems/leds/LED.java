package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Priority-based LED manager following README indicator definitions. */
public class LED extends SubsystemBase {
  private static final double FLASH_PERIOD_SEC = 0.25;
  // 4 flash periods = red on/off twice.
  private static final double INVALID_SHOT_FLASH_DURATION_SEC = FLASH_PERIOD_SEC * 4.0;

  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

  private boolean motorOverheated = false;
  private boolean motorDisconnected = false;
  private boolean beachAlertActive = false;
  private boolean autoAlignActive = false;
  private boolean intakeDown = false;
  private boolean hoodStowed = false;
  private double invalidShotFlashStartSec = Double.NEGATIVE_INFINITY;

  public LED(LEDIO io) {
    this.io = io;
  }

  public void setMotorOverheated(boolean motorOverheated) {
    this.motorOverheated = motorOverheated;
  }

  public void setMotorDisconnected(boolean motorDisconnected) {
    this.motorDisconnected = motorDisconnected;
  }

  public void setBeachAlertActive(boolean beachAlertActive) {
    this.beachAlertActive = beachAlertActive;
  }

  public void setAutoAlignActive(boolean autoAlignActive) {
    this.autoAlignActive = autoAlignActive;
  }

  public void setIntakeDown(boolean intakeDown) {
    this.intakeDown = intakeDown;
  }

  public void toggleIntakeDown() {
    intakeDown = !intakeDown;
  }

  public void setHoodStowed(boolean hoodStowed) {
    this.hoodStowed = hoodStowed;
  }

  public void triggerInvalidShotFlash() {
    // Restart the pulse sequence each time a new invalid-shot event is raised.
    invalidShotFlashStartSec = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LED", inputs);

    if (motorOverheated) {
      if (isFlashOn()) {
        io.setRGB(255, 255, 255); // White flashing
      } else {
        io.clear();
      }
      return;
    }

    if (motorDisconnected) {
      if (isFlashOn()) {
        io.setRGB(255, 165, 0); // Orange flashing
      } else {
        io.clear();
      }
      return;
    }

    if (beachAlertActive) {
      io.setRainbowFade();
      return;
    }

    // Flash red twice on an invalid shot, then fall back to the normal indicator priority.
    if (isInvalidShotFlashActive()) {
      if (isInvalidShotFlashOn()) {
        io.setRGB(255, 0, 0);
      } else {
        io.clear();
      }
      return;
    }

    if (autoAlignActive) {
      io.setRGB(255, 0, 0); // Red solid
      return;
    }

    if (intakeDown) {
      io.setRGB(0, 255, 0); // Green solid
      return;
    }

    if (hoodStowed) {
      io.setRGB(0, 0, 255); // Blue solid
      return;
    }

    io.clear();
  }

  private static boolean isFlashOn() {
    return ((int) (Timer.getFPGATimestamp() / FLASH_PERIOD_SEC)) % 2 == 0;
  }

  private boolean isInvalidShotFlashActive() {
    return Timer.getFPGATimestamp() - invalidShotFlashStartSec < INVALID_SHOT_FLASH_DURATION_SEC;
  }

  private boolean isInvalidShotFlashOn() {
    double elapsedSec = Timer.getFPGATimestamp() - invalidShotFlashStartSec;
    return ((int) (elapsedSec / FLASH_PERIOD_SEC)) % 2 == 0;
  }
}
