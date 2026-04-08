package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class LEDSubsystem extends SubsystemBase {

  public enum LEDState {
    RAINBOW,
    BLUE, //shooting
    GREEN, //ready to shoot
    YELLOW, //Spins the intake wheel & spindexer foreward
    ORANGE, //second level for climber
    RED, //Spins the intake wheel & spindexer backward
    TEAL 
  }

  private LEDState m_state = LEDState.RAINBOW;

  private final AddressableLED m_led = new AddressableLED(Constants.LEDS.PWM_PORT);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LEDS.NUM_LEDS);
  private final Distance kLedSpacing = Meters.of(1 / 60.0);

  private final LinearVelocity kScrollingSpeed = MetersPerSecond.of(0.2);

  private final Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
  private final LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtAbsoluteSpeed(kScrollingSpeed, kLedSpacing);

  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
  private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(kScrollingSpeed, kLedSpacing);
//the shooter is running
  private final LEDPattern m_blue = LEDPattern.solid(new Color(0, 0, 255));
  private final LEDPattern m_scrollingBlue = m_blue.mask(mask);

//the shooter is not running but is ready to
  private final LEDPattern m_green = LEDPattern.solid(new Color(0, 255, 0));
  private final LEDPattern m_scrollingGreen = m_green.mask(mask);

//Spins the intake wheel & spindexer foreward
  private final LEDPattern m_yellow = LEDPattern.solid(new Color(255, 160, 0));
  private final LEDPattern m_scrollingYellow = m_yellow.mask(mask);

//uh... this is being weird right now... in program for the second level of climber but isn't being used...
  //private final LEDPattern m_orange = LEDPattern.solid(new Color(255, 30, 0));
  //private final LEDPattern m_scrollingOrange = m_orange.mask(mask);

//Spins the intake wheel & spindexer backward
  private final LEDPattern m_red = LEDPattern.solid(new Color(255, 0, 0));
  private final LEDPattern m_scrollingRed = m_red.mask(mask);

//Team color (hopefully this is teal(ish...)) (hasn't been tested yet to know if it is the right color...)
  private final LEDPattern m_teal = LEDPattern.solid(new Color(0, 128, 128));
  private final LEDPattern m_scrollingteal = m_teal.mask(mask);



  public LEDSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setLEDState(LEDState state) {
    m_state = state;
  }

  public Command setLEDStateCommand(LEDState state) {
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
       case BLUE:
         m_scrollingBlue.applyTo(m_ledBuffer);
         break;
       case GREEN:
         m_scrollingGreen.applyTo(m_ledBuffer);
         break;
       case YELLOW:
         m_scrollingYellow.applyTo(m_ledBuffer);
         break;
       //case ORANGE:
        // m_scrollingOrange.applyTo(m_ledBuffer);
        // break;
       case RED:
         m_scrollingRed.applyTo(m_ledBuffer);
         break;
       default:
         m_scrollingRainbow.applyTo(m_ledBuffer);
         break;
     }

     // DriverStation State Overrides
     if (DriverStation.isDisabled()) {
       m_scrollingRainbow.applyTo(m_ledBuffer);
     } else if (DriverStation.isTest()) {
       m_scrollingRainbow.applyTo(m_ledBuffer);
     }

    SmartDashboard.putString("LED State", m_state.name());

    m_led.setData(m_ledBuffer);
  }
}