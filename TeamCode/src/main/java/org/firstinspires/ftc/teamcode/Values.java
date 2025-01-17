/*
 * Static values for Zodiac FTC
 */

package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.Map;

public class Values {
    Map<String, Integer> liftPositions = new HashMap<String, Integer>(){{
        put("zero", 0);
        put("up", 2000);
    }};

    Map<String, Double> clipPositions = new HashMap<String, Double>(){{
        put("DC_close", 0.29);
        put("DC_open", 0.55);
        put("TC_close", 0.77);
        put("TC_open", 0.55);
    }};

    Map<String, Double> armPositions = new HashMap<String, Double>(){{
        put("DC_arm_down", 0.0);
        put("DC_arm_change", 0.5);
    }};

    public final double chassisMultiplier = 1.1;
    public final double AutonomousCPower = 0.5;
}
