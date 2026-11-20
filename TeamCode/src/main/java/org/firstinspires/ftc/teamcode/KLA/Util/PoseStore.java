package org.firstinspires.ftc.teamcode.KLA.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

/** 오토 마지막 위치 → TeleOp 시작 위치. 앱이 켜져 있는 동안만 유지된다. */
@Configurable
public class PoseStore {
    /** 오토가 끝난 뒤 이 시간(초) 안에 TeleOp을 시작해야 이어받는다. 지나면 OPTIONS 위치로 시작. */
    public static double MAX_AGE_S = 120;

    private static Pose pose;
    private static Alliance alliance;
    private static long savedNs;

    /** 오토 루프에서 매번 호출. */
    public static void save(Pose p, Alliance a) {
        pose = p;
        alliance = a;
        savedNs = System.nanoTime();
    }

    /** 같은 진영 오토가 최근에 남긴 위치. 한 번 꺼내면 지워진다. 없으면 null. */
    public static Pose take(Alliance a) {
        Pose p = pose;
        boolean fresh = p != null && a == alliance && (System.nanoTime() - savedNs) / 1e9 <= MAX_AGE_S;
        pose = null;
        return fresh ? p : null;
    }
}
