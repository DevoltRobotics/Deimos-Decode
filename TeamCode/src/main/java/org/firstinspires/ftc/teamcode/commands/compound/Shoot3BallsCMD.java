package org.firstinspires.ftc.teamcode.commands.compound;


import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.commands.intake.IntakeInCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.ShootAllCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.ShootModeCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.SpindexPosCMD;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

import java.util.function.DoubleSupplier;

public class Shoot3BallsCMD extends ParallelDeadlineGroup {

    SpindexSubsystem spindexSubsystem;

    public Shoot3BallsCMD(SpindexSubsystem spindexSubsystem, IntakeSubsystem intakeSubsystem, DoubleSupplier startingSpindex) {
        super(new SequentialCommandGroup() {
            @Override
            public void initialize() {
                if (startingSpindex != null) {
                    addCommands(
                            new SpindexPosCMD(spindexSubsystem, startingSpindex)
                    );
                } else {
                    addCommands(
                            new ShootModeCMD(spindexSubsystem)
                    );
                }
                addCommands(
                        new ShootAllCMD(spindexSubsystem),
                        new InstantCommand(()-> spindexSubsystem.setnBalls(0))
                );

                super.initialize();
            }
        }, new IntakeInCMD(intakeSubsystem));

        this.spindexSubsystem = spindexSubsystem;
    }

    public Shoot3BallsCMD(SpindexSubsystem spindexSubsystem, IntakeSubsystem intakeSubsystem) {
        this(spindexSubsystem, intakeSubsystem, null);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        if(interrupted) {
            spindexSubsystem.setShooting(false);
        }
    }
}
