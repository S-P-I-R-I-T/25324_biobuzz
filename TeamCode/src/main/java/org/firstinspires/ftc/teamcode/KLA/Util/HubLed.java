package org.firstinspires.ftc.teamcode.KLA.Util;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/** 허브 LED 무지개. 허브 자체가 패턴을 반복하므로 한 번만 호출. 끝날 때 restore()로 원래 색 복구. */
@Configurable
public class HubLed {
    /** 색 하나가 켜져 있는 시간(ms). 작을수록 빨리 돈다. */
    public static int STEP_MS = 120;

    public static void rainbow(HardwareMap hardwareMap) {
        shownReady = false;
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.pushPattern(rainbowPattern(hub));   // 원래 패턴은 스택에 보관
        }
    }

    private static Boolean shownReady = null;

    /** 발사 준비 표시: true = 초록 고정, false = 무지개. 바뀔 때만 허브에 보냄. */
    public static void showReady(HardwareMap hardwareMap, boolean ready) {
        if (shownReady != null && shownReady == ready) return;
        shownReady = ready;
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            if (ready) hub.setConstant(Color.GREEN);
            else hub.setPattern(rainbowPattern(hub));
        }
    }

    private static List<Blinker.Step> rainbowPattern(LynxModule hub) {
        int steps = Math.max(2, Math.min(16, hub.getBlinkerPatternMaxLength()));
        List<Blinker.Step> pattern = new ArrayList<>();
        for (int i = 0; i < steps; i++) {
            int color = Color.HSVToColor(new float[]{360f * i / steps, 1f, 1f});
            pattern.add(new Blinker.Step(color, STEP_MS, TimeUnit.MILLISECONDS));
        }
        return pattern;
    }

    /** rainbow() 이전 색으로 되돌린다. */
    public static void restore(HardwareMap hardwareMap) {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            if (hub.patternStackNotEmpty()) hub.popPattern();
        }
        shownReady = null;
    }
}
