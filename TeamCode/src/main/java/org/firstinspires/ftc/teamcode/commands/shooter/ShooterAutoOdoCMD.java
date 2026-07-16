package org.firstinspires.ftc.teamcode.commands.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class ShooterAutoOdoCMD extends CommandBase {

    private static InterpLUT VelocityLUT = new InterpLUT();
    private static  InterpLUT AngleLUT = new InterpLUT();

    static {
        VelocityLUT.add(0, 0);

        VelocityLUT.add(36, 1100);
        VelocityLUT.add(54, 1130);
        VelocityLUT.add(88, 1240);
        VelocityLUT.add(92.66,1275);
        VelocityLUT.add(122, 1420);
        VelocityLUT.add(160, 1570); // new
        VelocityLUT.add(205, 1640);

        VelocityLUT.add(1000000000, 10000);


        VelocityLUT.createLUT();

        AngleLUT.add(0,0);

        AngleLUT.add(36,0.1);
        AngleLUT.add(37,0.2);
        AngleLUT.add(38,0.3);
        AngleLUT.add(39,0.4);
        AngleLUT.add(40,0.5);
        AngleLUT.add(41,0.6);
        AngleLUT.add(42,0.7);
        AngleLUT.add(43,0.8);

        AngleLUT.add(1000000000,100000);

        AngleLUT.createLUT();

    }



    double distance;
    private final ShooterSubsystem subsystem;

    private final TurretSubsystem turretSubsystem;

    public ShooterAutoOdoCMD(ShooterSubsystem subsystem, TurretSubsystem turretSubsystem) {
        this.subsystem = subsystem;
        this.turretSubsystem = turretSubsystem;
        addRequirements(subsystem);

    }

    @Override
    public void execute(){

        distance = turretSubsystem.getDistanceToGoal();

        double velocity = VelocityLUT.get(distance);
        double HoodPos = AngleLUT.get(distance);

        subsystem.setTargetVelocity(Range.clip(velocity, 1100,1570));
        subsystem.setHood(Range.clip(HoodPos,0,1));


    }
}
