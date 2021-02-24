package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Turret;

public class ExtendIntake extends SequentialCommandGroup {
    public ExtendIntake(Intake intake) {

        addCommands(
                new IntakeSetSpeed(intake, 1.0),
                new WaitCommand(0.5),
                new IntakeSetSpeed(intake, 0)
        );
    }
}
