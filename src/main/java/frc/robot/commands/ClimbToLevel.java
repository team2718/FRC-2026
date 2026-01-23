package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public final class ClimbToLevel extends Command {
    // Data Fields
    private ClimberSubsystem climber;
    private int level;

    // Constructors
    public ClimbToLevel(ClimberSubsystem climber, int level){
        this.climber = climber;
        this.level = level;
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
                climber.setClimbMotor((Constants.ClimberConstants.BAR1_ELEVATION - climber.getHookElevation()) * Constants.ClimberConstants.CLIMBER_P);
            }
            else {
                climber.setClimbMotor((Constants.ClimberConstants.BAR1_ELEVATION - climber.getHookElevation()) * -1 * Constants.ClimberConstants.CLIMBER_P);
            }
        }
        // L2 Climb
        else if ((climber.getDesiredLevel() == 2 && climber.getRobotElevation() < Constants.ClimberConstants.L2_ELEVATION) ||
        (climber.getDesiredLevel() > 2 && climber.getRobotElevation() < Constants.ClimberConstants.BAR2_ELEVATION)) {
            if (climber.getHookElevation() < Constants.ClimberConstants.BAR2_ELEVATION) {
                climber.setClimbMotor((Constants.ClimberConstants.BAR2_ELEVATION - climber.getHookElevation()) * Constants.ClimberConstants.CLIMBER_P);
            }
            else {
                climber.setClimbMotor((Constants.ClimberConstants.BAR2_ELEVATION - climber.getHookElevation()) * -1 * Constants.ClimberConstants.CLIMBER_P);
            }
        }
        // L3 Climb
        else if (climber.getDesiredLevel() == 3 && climber.getRobotElevation() < Constants.ClimberConstants.L3_ELEVATION) {
            if (climber.getHookElevation() < Constants.ClimberConstants.BAR3_ELEVATION) {
                climber.setClimbMotor((Constants.ClimberConstants.BAR3_ELEVATION - climber.getHookElevation()) * Constants.ClimberConstants.CLIMBER_P);
            }
            else {
                climber.setClimbMotor((Constants.ClimberConstants.BAR3_ELEVATION - climber.getHookElevation()) * -1 * Constants.ClimberConstants.CLIMBER_P);
            }    
        }
    }
}
