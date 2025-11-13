package org.firstinspires.ftc.teamcode.commands.sorter;

import static org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem.deltaRad;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class NextPosSorterCMD extends CommandBase {

    private final SpindexSubsystem spindexSubsystem;
    private final boolean reverse;

    // Recorremos ~120°; dejamos margen para no exigir exacto 120
    private static final double REQUIRED_TRAVEL_DEG = 116.0; // ajuste fino si quieres 114–118
    private static final double REQUIRED_TRAVEL_RAD = Math.toRadians(REQUIRED_TRAVEL_DEG);

    // Estado interno
    private double startPosRad;     // posición al iniciar
    private double lastPosRad;      // última posición muestreada
    private double targetPosRad;    // solo para definir dirección
    private double dirSign;         // +1 si el objetivo está CCW desde start, -1 si CW
    private double traveledRad;     // avance acumulado SOLO en la dirección correcta

    public NextPosSorterCMD(SpindexSubsystem spindexSubsystem, boolean reverse){
        this.spindexSubsystem = spindexSubsystem;
        this.reverse = reverse;
        addRequirements(spindexSubsystem);
    }

    @Override
    public void initialize(){
        // Captura posición inicial ANTES de pedir el salto
        startPosRad = spindexSubsystem.getCurrentRad();
        lastPosRad  = startPosRad;
        traveledRad = 0.0;

        // Pide el siguiente sector (±120°)
        spindexSubsystem.nextPos(reverse);

        // Define la dirección esperada mirando de start hacia el nuevo target
        targetPosRad = spindexSubsystem.getTargetRad();
        double wrappedToTarget = deltaRad(targetPosRad, startPosRad); // (-π, π]
        dirSign = Math.signum(wrappedToTarget);
        if (dirSign == 0.0) {
            // Caso rarísimo: si por redondeo quedó 0, usa el parámetro reverse para decidir
            dirSign = reverse ? +1.0 : -1.0;
        }
    }

    @Override
    public void execute(){
        double curr = spindexSubsystem.getCurrentRad();

        // Delta envuelto entre lecturas consecutivas
        double step = deltaRad(curr, lastPosRad); // (-π, π]

        // Proyecta ese paso en la dirección esperada; ignora retrocesos/backlash
        double stepAlong = step * dirSign;
        if (stepAlong > 0.0) {
            traveledRad += stepAlong;
        }

        lastPosRad = curr;
    }

    @Override
    public boolean isFinished(){
        // Termina cuando haya avanzado lo suficiente en la dirección del target
       return traveledRad >= REQUIRED_TRAVEL_RAD;

    }
}
