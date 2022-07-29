package org.firstinspires.ftc.teamcode.roadrunnerext;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

@Config
public class RamseteConstants {
    public static double b = 5.0;
    public static double zeta = 0.7;
    public static PIDCoefficients linearVeloPID = new PIDCoefficients(0.7, 0.0, 0.0);
    public static PIDCoefficients angularVeloPID = new PIDCoefficients(2.0, 0.0, 0.0);
}
