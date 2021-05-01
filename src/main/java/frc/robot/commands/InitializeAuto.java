package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;

public class InitializeAuto extends SequentialCommandGroup {
    Climb mClimb = Climb.getInstance();

    public InitializeAuto(Intake intake, Climb climb) {
            addCommands(
                    new ParallelCommandGroup(
                            new IntakeSetRPM(intake, Constants.INTAKE_COLLECT_RPM),
                            new ClimbSetSpeed(climb, -3.0)
                    )
            );

    }
}
