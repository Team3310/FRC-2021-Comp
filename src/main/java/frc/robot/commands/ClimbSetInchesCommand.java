package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Turret;


public class ClimbSetInchesCommand extends SequentialCommandGroup {
    Climb mClimb = Climb.getInstance();

    public ClimbSetInchesCommand(double inches) {
        mClimb.setClimbMotionMagicPositionAbsolute(inches);
    }
}
