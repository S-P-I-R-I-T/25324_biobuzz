package org.firstinspires.ftc.teamcode.KLA.Util;

import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.Mechanism;

import java.util.Set;

/** Robot declaration discovered by NextFTC. */
public class Robot implements NextRobot {
    public Action action;
    public Set<Mechanism> mechanisms = Set.of();

    public Robot() { }

    @Override public Set<Mechanism> getMechanisms() { return mechanisms; }
}
