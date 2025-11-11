package org.firstinspires.ftc.teamcode.commands.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterAutoLLCMD extends CommandBase {

    private static InterpLUT VelocityLUT = new InterpLUT();

    static {
        VelocityLUT.add(0.12, 1900);
        VelocityLUT.add(0.2, 1670);
        VelocityLUT.add(0.25, 1565);
        VelocityLUT.add(0.4, 1430);
        VelocityLUT.add(0.713, 1300);
        VelocityLUT.add(3.22, 1050);
        VelocityLUT.add(10, 0);


        VelocityLUT.createLUT();
    }

    private final ShooterSubsystem subsystem;

    private final LLSubsystem LL;

    public ShooterAutoLLCMD(ShooterSubsystem subsystem, LLSubsystem LL) {
        this.subsystem = subsystem;
        this.LL = LL;  
        addRequirements(subsystem);
    }

    @Override
    public void execute(){

        if (LL.result != null && LL.result.isValid()) {
            Double Area = LL.getAllianceTA();
            if(Area != null){
                double velocity = VelocityLUT.get(Area);

                subsystem.setTargetVelocity(Range.clip(velocity, 1050,1565));
            }
        }
    }
}
