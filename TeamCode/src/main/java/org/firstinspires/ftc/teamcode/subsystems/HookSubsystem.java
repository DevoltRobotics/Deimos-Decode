package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class HookSubsystem extends SubsystemBase {

    public Servo hook;
    public int nFlick = 0;

    public HookSubsystem(HardwareMap hMap){
        hook = hMap.get(Servo.class,"gancho");

    }

    public void hookUp(){
        hook.setPosition(0);
    }

    public void addnFLick(){
        nFlick += 1;
    }

    public void hookDown(){
        hook.setPosition(0.5);
    }
}
