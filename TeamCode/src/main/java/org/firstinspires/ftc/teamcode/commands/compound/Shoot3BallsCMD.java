package org.firstinspires.ftc.teamcode.commands.compound;


import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.sorter.NextPosSorterCMD;
import org.firstinspires.ftc.teamcode.commands.hook.UpAndDownCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.ShootModeCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.SpindexPosCMD;
import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

import java.util.function.DoubleSupplier;

public class Shoot3BallsCMD extends SequentialCommandGroup {
    SpindexSubsystem spindexSubsystem;
    public Shoot3BallsCMD(HookSubsystem hookSubsystem, SpindexSubsystem spindexSubsystem, DoubleSupplier startingSpindex) {
        this.spindexSubsystem = spindexSubsystem;
        if (startingSpindex != null) {
            addCommands(new InstantCommand(() -> spindexSubsystem.setShooting(true)),new SpindexPosCMD(spindexSubsystem, startingSpindex)
                    );
        } else {
            addCommands(new InstantCommand(() -> spindexSubsystem.setShooting(true)),new ShootModeCMD(spindexSubsystem)
                    );
        }

        addCommands(
                new InstantCommand(() -> spindexSubsystem.setShooting(true)),
                new WaitCommand(350),

                new UpAndDownCMD(hookSubsystem, spindexSubsystem),
                new NextPosSorterCMD(spindexSubsystem),

                new WaitCommand(200),

                new UpAndDownCMD(hookSubsystem, spindexSubsystem),
                new NextPosSorterCMD(spindexSubsystem),

                new WaitCommand(200),

                new UpAndDownCMD(hookSubsystem, spindexSubsystem),
                new InstantCommand(() -> spindexSubsystem.setShooting(false))
        );


    }




    public Shoot3BallsCMD(HookSubsystem hookSubsystem, SpindexSubsystem spindexSubsystem) {
        this(hookSubsystem, spindexSubsystem, null);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if(interrupted) {
            spindexSubsystem.setShooting(false);
        }
    }
}
