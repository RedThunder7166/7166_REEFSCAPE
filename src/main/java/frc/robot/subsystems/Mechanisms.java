// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class Mechanisms {
    public static final class ElevatorMechanisms {
        public static final double baseHeightInches = 38.875;
        public static final double baseHeightMeters = Units.inchesToMeters(baseHeightInches);
        public static final double maxHeightInches = 65.25 - baseHeightInches;

        public static Mechanism2d mechanism2d = new Mechanism2d(Units.inchesToMeters(26), baseHeightMeters);
        public static MechanismRoot2d root2d = mechanism2d.getRoot("Elevator", Units.inchesToMeters(16.5), 0);
        public static MechanismLigament2d ligament = root2d.append(new MechanismLigament2d("elevator", 0, 90));
    }
    public static final class GantryMechanisms {
        public static final double widthInches = 23.472;
        public static MechanismLigament2d ligament = ElevatorMechanisms.ligament.append(new MechanismLigament2d("gantry", 0, 90));
    }
    public static final class AlgaeHandMechanisms {
        public static MechanismLigament2d ligament = ElevatorMechanisms.ligament.append(new MechanismLigament2d("algaehand", 0, 45));
    }
}
