package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auto.commands.ResetOdometryAuto;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Turret;

public class InitializeAuto extends SequentialCommandGroup {
    Climb mClimb = Climb.getInstance();

    public InitializeAuto(Intake intake) {
            addCommands(
                    new ParallelCommandGroup(
                            new IntakeSetRPM(intake, Constants.INTAKE_COLLECT_RPM),
                            new ClimbSetInchesCommand(-3.0)
                    )
            );

    }
}
