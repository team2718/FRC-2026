package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import org.ejml.masks.Mask;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SK6812RGBW;

public class LEDSubsystem extends SubsystemBase {

  public enum LEDState {
    RAINBOW,
    SHOOTER
  }

  private LEDState m_state = LEDState.RAINBOW;

  private final SK6812RGBW m_led = new SK6812RGBW(Constants.LEDS.PWMPort);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LEDS.Length);
  private final Distance kLedSpacing = Meters.of(1 / 60.0);

  private final LinearVelocity kScrollingSpeed = MetersPerSecond.of(0.5);

  private final Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
  private final LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtAbsoluteSpeed(kScrollingSpeed, kLedSpacing);

  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
  private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(kScrollingSpeed, kLedSpacing);

  private final LEDPattern m_shooter = LEDPattern.solid(new Color(255, 255, 0));
  private final LEDPattern m_scrollingShooter = m_shooter.mask(mask);


  public LEDSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public Command setLEDState(LEDState state) {
    return Commands.runOnce(() -> {
        m_state = state;
    });
  }

  @Override
  public void periodic() {
    switch (m_state) {
      case RAINBOW:
        m_scrollingRainbow.applyTo(m_ledBuffer);
        break;
      case SHOOTER:
        m_scrollingShooter.applyTo(m_ledBuffer);
        break;
      default:
        m_scrollingRainbow.applyTo(m_ledBuffer);
        break;
    }

    // DriverStation State Overrides
    if (DriverStation.isDisabled()) {
        
    } else if (DriverStation.isTest()) {
      m_scrollingRainbow.applyTo(m_ledBuffer);
    }

    m_led.setData(m_ledBuffer);
  }
}