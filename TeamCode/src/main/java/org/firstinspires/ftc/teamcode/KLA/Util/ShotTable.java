package org.firstinspires.ftc.teamcode.KLA.Util;

import com.bylazar.configurables.annotations.Configurable;

import java.util.Locale;

/**
 * 목표까지 거리(inch) → 후드. 속도는 Action.SHOOTING_VELOCITY 고정.
 * 점들을 그대로 잇지 않고 최소제곱 곡선으로 맞춘다 (측정 오차에 덜 흔들림). 순서 상관없음.
 * 측정 범위 밖 거리는 범위 끝 값으로 고정.
 */
@Configurable
public class ShotTable {
    public static double[] DISTANCE = {33.4,38.5,38.3,44.6};
    public static double[] HOOD = {0.855,0.85,0.85,0.85};
    /** 1 = 직선, 2 = 곡선. 점 개수가 부족하면 자동으로 낮춘다. */
    public static int FIT_DEGREE = 2;

    public static int count() { return Math.min(DISTANCE.length, HOOD.length); }
    public static boolean valid() { return count() > 0; }

    public static double hood(double distance) {
        int n = count();
        if (n == 0) return Double.NaN;
        double min = DISTANCE[0], max = DISTANCE[0], mean = 0;
        for (int i = 0; i < n; i++) {
            min = Math.min(min, DISTANCE[i]);
            max = Math.max(max, DISTANCE[i]);
            mean += DISTANCE[i] / n;
        }
        double x = Math.max(min, Math.min(max, distance)) - mean;
        double[] c = fit(n, mean, Math.max(0, Math.min(FIT_DEGREE, n - 1)));
        double y = 0, p = 1;
        for (double ci : c) { y += ci * p; p *= x; }
        return y;
    }

    /** 최소제곱 다항식 (x는 평균 중심화). */
    private static double[] fit(int n, double mean, int deg) {
        while (true) {
            int m = deg + 1;
            double[][] a = new double[m][m + 1];
            for (int i = 0; i < n; i++) {
                double x = DISTANCE[i] - mean;
                for (int r = 0; r < m; r++) {
                    for (int col = 0; col < m; col++) a[r][col] += Math.pow(x, r + col);
                    a[r][m] += HOOD[i] * Math.pow(x, r);
                }
            }
            double[] c = solve(a, m);
            if (c != null || deg == 0) return c != null ? c : new double[]{0};
            deg--;   // 같은 거리 점만 있는 경우 등 → 차수 낮춤
        }
    }

    private static double[] solve(double[][] a, int m) {
        for (int i = 0; i < m; i++) {
            int piv = i;
            for (int r = i + 1; r < m; r++) if (Math.abs(a[r][i]) > Math.abs(a[piv][i])) piv = r;
            if (Math.abs(a[piv][i]) < 1e-9) return null;
            double[] t = a[i]; a[i] = a[piv]; a[piv] = t;
            for (int r = 0; r < m; r++) {
                if (r == i) continue;
                double f = a[r][i] / a[i][i];
                for (int col = i; col <= m; col++) a[r][col] -= f * a[i][col];
            }
        }
        double[] c = new double[m];
        for (int i = 0; i < m; i++) c[i] = a[i][m] / a[i][i];
        return c;
    }

    // ── TeleOp에서 기록 ──
    public static void add(double distance, double hood) {
        int n = count();
        double[] d = new double[n + 1], h = new double[n + 1];
        System.arraycopy(DISTANCE, 0, d, 0, n);
        System.arraycopy(HOOD, 0, h, 0, n);
        d[n] = round(distance, 1);
        h[n] = round(hood, 3);
        DISTANCE = d;
        HOOD = h;
    }

    public static void removeLast() {
        int n = count();
        if (n == 0) return;
        DISTANCE = java.util.Arrays.copyOf(DISTANCE, n - 1);
        HOOD = java.util.Arrays.copyOf(HOOD, n - 1);
    }

    /** 코드에 그대로 붙여넣을 줄. */
    public static String code() {
        StringBuilder d = new StringBuilder(), h = new StringBuilder();
        for (int i = 0; i < count(); i++) {
            if (i > 0) { d.append(", "); h.append(", "); }
            d.append(String.format(Locale.US, "%.1f", DISTANCE[i]));
            h.append(String.format(Locale.US, "%.3f", HOOD[i]));
        }
        return "DISTANCE = {" + d + "};  HOOD = {" + h + "};";
    }

    private static double round(double v, int digits) {
        double s = Math.pow(10, digits);
        return Math.round(v * s) / s;
    }
}
