package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public final class ClimbToLevel extends Command {
    // Data Fields
    private ClimberSubsystem climber;
    private int level;

    private final LEDSubsystem led;

    // Constructors
    public ClimbToLevel(ClimberSubsystem climber, int level, LEDSubsystem led){
        this.climber = climber;
        this.level = level;
        this.led = led;
        addRequirements(climber);

        led = new LEDSubsystem();

    }

    // Method Definition
    @Override
    public void initialize() {
        if (!climber.isClimbing() && !climber.isReleasing()) {
            climber.setClimbing(true);
            climber.setDesiredLevel(level);
        }
    }

    @Override
    public void execute() {
        // L1 Climb
        if ((climber.getDesiredLevel() == 1 && climber.getRobotElevation() < Constants.ClimberConstants.L1_ELEVATION) || 
        (climber.getDesiredLevel() > 1 && climber.getRobotElevation() < Constants.ClimberConstants.BAR1_ELEVATION)) {
            if (climber.getHookElevation() < Constants.ClimberConstants.BAR1_ELEVATION) {
                climber.setClimbMotor((Constants.ClimberConstants.BAR1_ELEVATION - climber.getHookElevation()) * Constants.ClimberConstants.EXTEND_P);
                //yellow LEDs = first level
                led.setLEDState(LEDState.YELLOW);
            }
            else {
                climber.setClimbMotor((Constants.ClimberConstants.BAR1_ELEVATION - climber.getHookElevation()) * -1 * Constants.ClimberConstants.RETRACT_P);
            }
        }
        // L2 Climb
        else if ((climber.getDesiredLevel() == 2 && climber.getRobotElevation() < Constants.ClimberConstants.L2_ELEVATION) ||
        (climber.getDesiredLevel() > 2 && climber.getRobotElevation() < Constants.ClimberConstants.BAR2_ELEVATION)) {
            if (climber.getHookElevation() < Constants.ClimberConstants.BAR2_ELEVATION) {
                climber.setClimbMotor((Constants.ClimberConstants.BAR2_ELEVATION - climber.getHookElevation()) * Constants.ClimberConstants.EXTEND_P);
                //orange LEDs = second level
                led.setLEDState(LEDState.ORANGE);
            }
            else {
                climber.setClimbMotor((Constants.ClimberConstants.BAR2_ELEVATION - climber.getHookElevation()) * -1 * Constants.ClimberConstants.RETRACT_P);
            }
        }
        // L3 Climb
        else if (climber.getDesiredLevel() == 3 && climber.getRobotElevation() < Constants.ClimberConstants.L3_ELEVATION) {
            if (climber.getHookElevation() < Constants.ClimberConstants.BAR3_ELEVATION) {
                climber.setClimbMotor((Constants.ClimberConstants.BAR3_ELEVATION - climber.getHookElevation()) * Constants.ClimberConstants.EXTEND_P);
                //red LEDs = third level
                led.setLEDState(LEDState.RED);
            }
            else {
                climber.setClimbMotor((Constants.ClimberConstants.BAR3_ELEVATION - climber.getHookElevation()) * -1 * Constants.ClimberConstants.RETRACT_P);
            }    
        }
    }

    @Override
    public boolean isFinished() {
        return (
            climber.getDesiredLevel() == 0 ||
            (climber.getDesiredLevel() == 1 && climber.getRobotElevation() >= Constants.ClimberConstants.L1_ELEVATION) ||
            (climber.getDesiredLevel() == 2 && climber.getRobotElevation() >= Constants.ClimberConstants.L2_ELEVATION) ||
            (climber.getDesiredLevel() == 3 && climber.getRobotElevation() >= Constants.ClimberConstants.L3_ELEVATION)
        );
    }

    @Override
    public void end(boolean interuppted) {
        climber.setClimbMotor(0);
    }
}
