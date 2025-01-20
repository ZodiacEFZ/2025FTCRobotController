/*
 * Static values for Zodiac FTC
 */

package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.Map;

public class Values {
    Map<String, Double> speed = new HashMap<String, Double>(){{
        put("down_clip_arm_ly2", 0.075);
    }};

    Map<String, Integer> intSpeed = new HashMap<String, Integer>(){{
        put("intake", 60);
    }};

    Map<String, Integer> liftPositions = new HashMap<String, Integer>(){{
        put("zero", 0);
        put("up", 620);
        put("put", 1600);
        put("max", 1800);
    }};

    /*Map<String, Integer> intakePositions = new HashMap<String, Integer>(){{

    }};*/

    Map<String, Double> headPositions = new HashMap<String, Double>(){{
        put("Head_down", 0.4);
        put("Head_up", 1.0);
    }};

    Map<String, Double> clipPositions = new HashMap<String, Double>(){{
        put("DC_close", 0.29);
        put("DC_open", 0.55);
        put("TC_close", 0.71);
        put("TC_open", 0.55);
    }};

    Map<String, Double> armPositions = new HashMap<String, Double>(){{
        put("DC_arm_down", 0.0);
        //put("DC_arm_change", 0.5);
        put("DC_arm_up", 0.7);
        put("DC_arm_init", 1.0);
        put("DC_head_init", 0.5);
        //put("DC_arm_init", 1.0);
        put("TC_arm_up", 1.0);
        put("TC_arm_down", 0.0);
    }};

    public final double chassisMultiplier = 1.1;
    public final double AutonomousCPower = 0.5;
}
