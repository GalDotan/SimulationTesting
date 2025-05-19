
package frc.robot.RobotControl;



import frc.robot.RobotContainer;
import frc.robot.Subsystem.Gripper.Gripper;
import frc.robot.Subsystem.Intake.IntakeRoller.IntakeRoller;

public class SuperStructure {

    public static IntakeRoller intakeRoller;
    public static Gripper gripper;
    private static Field.ScoringLevel scoringLevel;

    public SuperStructure() { //TODO: GENERIC SUPER STRUCTURE
        intakeRoller = RobotContainer.intakeRoller;
        gripper = RobotContainer.gripper;

        scoringLevel = Field.ScoringLevel.L3;
    }

    public static boolean hasGamePiece() {
        return intakeRoller.hasCoral() || gripper.hasCoral();
    }

    public static void setScoringLevel(Field.ScoringLevel ScoringLevel) {
        scoringLevel = ScoringLevel;
    }

    public static Field.ScoringLevel getScoringLevel() {
        return scoringLevel;
    }

    public void update() {
    }


}
