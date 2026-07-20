package org.firstinspires.ftc.teamcode.commands.sorter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

@Config
public class ShootAllCMD extends CommandBase {
    private final SpindexSubsystem spindex;
    private final ElapsedTime timer = new ElapsedTime();
    private static MotionProfile profile;

    public static double maxAccel = 20;
    public static double maxJerk = 5;

    public ShootAllCMD(SpindexSubsystem spindex) {
        this.spindex = spindex;
        addRequirements(spindex);
    }

    @Override
    public void initialize() {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(-0.8, 0, 0),
                new MotionState(0, 0, 0),
                maxAccel,
                maxJerk
        );

        timer.reset();
        spindex.setShooting(true);
    }

    @Override
    public void execute(){
        double now = timer.seconds();
        double power = profile.get(now).getX();

        spindex.ShootPower = power;
    }

    @Override
    public void end(boolean interrupted) {
        spindex.setShooting(false);
        spindex.ShootPower = 0;
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= 1;
    }


}
