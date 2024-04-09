package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.RobotState.RobotCurrentAction;

public class DynamicCurrentLimitsManager {
    // singleton instance
    private static DynamicCurrentLimitsManager instance = null;
    private static double CURRENT_BUDGET = 340;
    private static int currentBrownoutCount = 0;
    private PowerDistribution pdp;

    // states for each ratio

    public RobotCurrentAction[] driveActions = {
        RobotCurrentAction.kStow,RobotCurrentAction.kAmpLineup,RobotCurrentAction.kAutoRevAndAutoAlign,RobotCurrentAction.kPathPlanner
    };
    public RobotCurrentAction[] shootingActions = {
        RobotCurrentAction.kRevAndAlign,RobotCurrentAction.kAutoRevAndAutoAlign,RobotCurrentAction.kAutoSOTM,RobotCurrentAction.kNothing,RobotCurrentAction.kAutoSOTM, RobotCurrentAction.kAutoShootAtPosition
    };
    public RobotCurrentAction[] intakeActions = {
        RobotCurrentAction.kIntake,RobotCurrentAction.kAutoIntake,RobotCurrentAction.kGamePieceLock
    };


    // private constructor
    private DynamicCurrentLimitsManager() {
        pdp = new PowerDistribution();

    }

    // get instance
    public static DynamicCurrentLimitsManager getInstance() {
        if (instance == null) {
            instance = new DynamicCurrentLimitsManager();
        }
        return instance;
    }

    public void update(){
        // frc get brownout count
        
        if(pdp.getFaults().Brownout){
            currentBrownoutCount++;
            CURRENT_BUDGET = Math.max(CURRENT_BUDGET - 10,150);
        }

        // if robot is in a drive action
        
        for (RobotCurrentAction action : driveActions) {
            if (frc.robot.RobotState.getInstance().curAction == action) {
                double moduleBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.7/4),30);
                double shooterBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.15),25);
                double shooterPivotBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.05),10);
                
                frc.robot.RobotState.getInstance().setCurrentLimits(moduleBudget, shooterBudget, shooterPivotBudget);
            }
        }

        // if robot is in a shooting action
        for (RobotCurrentAction action : shootingActions) {
            if (frc.robot.RobotState.getInstance().curAction == action) {
                double moduleBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.5/4),20);
                double shooterBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.4),50);
                double shooterPivotBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.1),15);
                
                frc.robot.RobotState.getInstance().setCurrentLimits(moduleBudget, shooterBudget, shooterPivotBudget);
            }
        }

        // if robot is in an intake action
        for (RobotCurrentAction action : intakeActions) {
            if (frc.robot.RobotState.getInstance().curAction == action) {
                double moduleBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.8/4),35);
                double shooterBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.1),25);
                double shooterPivotBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.05),15);
                
                frc.robot.RobotState.getInstance().setCurrentLimits(moduleBudget, shooterBudget, shooterPivotBudget);
            }
        }


        

    }

}
