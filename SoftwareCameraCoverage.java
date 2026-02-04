import java.util.*;

/**
 * Checks if a set of hardware cameras (rectangles in distance x light)
 * fully covers the required software camera rectangle.
 *
 * Distance axis: subject distance
 * Light axis: light level
 */
public class SoftwareCameraCoverage {

    // Small epsilon for floating comparisons (in case you use doubles).
    private static final double EPS = 1e-12;

    public static final class Camera {
        public final double dMin, dMax;  // distance range
        public final double lMin, lMax;  // light range

        public Camera(double dMin, double dMax, double lMin, double lMax) {
            if (dMin > dMax) throw new IllegalArgumentException("dMin > dMax");
            if (lMin > lMax) throw new IllegalArgumentException("lMin > lMax");
            this.dMin = dMin;
            this.dMax = dMax;
            this.lMin = lMin;
            this.lMax = lMax;
        }

        @Override public String toString() {
            return "Camera[d=(" + dMin + "," + dMax + "), l=(" + lMin + "," + lMax + ")]";
        }
    }

    // Internal: clipped camera intervals
    private static final class ClippedCam {
        final double dMin, dMax, lMin, lMax;
        ClippedCam(double dMin, double dMax, double lMin, double lMax) {
            this.dMin = dMin; this.dMax = dMax; this.lMin = lMin; this.lMax = lMax;
        }
    }

    private static final class Event {
        final double x;         // distance coordinate
        final boolean isStart;  // start=true, end=false
        final int camId;        // index into clipped list
        Event(double x, boolean isStart, int camId) {
            this.x = x; this.isStart = isStart; this.camId = camId;
        }
    }

    private static final class Interval {
        final double start, end;
        Interval(double start, double end) { this.start = start; this.end = end; }
    }

    /**
     * Returns true if the union of hardware camera rectangles covers the required rectangle:
     * distance in [reqDMin, reqDMax] and light in [reqLMin, reqLMax].
     */
    public static boolean camerasSuffice(
            double reqDMin, double reqDMax,
            double reqLMin, double reqLMax,
            List<Camera> hardware
    ) {
        if (reqDMin > reqDMax) throw new IllegalArgumentException("reqDMin > reqDMax");
        if (reqLMin > reqLMax) throw new IllegalArgumentException("reqLMin > reqLMax");

        // Degenerate required rectangle (zero area) is trivially covered.
        // If you want different semantics, adjust here.
        if (almostEqual(reqDMin, reqDMax) || almostEqual(reqLMin, reqLMax)) {
            return true;
        }

        // 1) Clip hardware cameras to the required rectangle; ignore empty intersections.
        List<ClippedCam> clipped = new ArrayList<>();
        for (Camera c : hardware) {
            double d1 = Math.max(reqDMin, c.dMin);
            double d2 = Math.min(reqDMax, c.dMax);
            double l1 = Math.max(reqLMin, c.lMin);
            double l2 = Math.min(reqLMax, c.lMax);

            if (d1 + EPS < d2 && l1 + EPS < l2) { // strict area intersection (avoid empty/line)
                clipped.add(new ClippedCam(d1, d2, l1, l2));
            }
        }

        if (clipped.isEmpty()) return false;

        // 2) Build sweep events over distance and coordinate boundaries.
        List<Event> events = new ArrayList<>(2 * clipped.size());
        ArrayList<Double> coords = new ArrayList<>(2 * clipped.size() + 2);
        coords.add(reqDMin);
        coords.add(reqDMax);

        for (int i = 0; i < clipped.size(); i++) {
            ClippedCam cc = clipped.get(i);
            events.add(new Event(cc.dMin, true, i));   // start
            events.add(new Event(cc.dMax, false, i));  // end
            coords.add(cc.dMin);
            coords.add(cc.dMax);
        }

        // Sort and unique coordinates
        coords.sort(Double::compare);
        coords = unique(coords);

        // Sort events by x, but at same x:
        // process END before START so intervals ending at x aren't active for slab starting at x (half-open slabs).
        events.sort((a, b) -> {
            int cmp = Double.compare(a.x, b.x);
            if (cmp != 0) return cmp;
            // end first
            if (a.isStart == b.isStart) return 0;
            return a.isStart ? 1 : -1;
        });

        // Active set of camera indices.
        // Using boolean[] + int list avoids heavy HashSet churn.
        boolean[] active = new boolean[clipped.size()];
        int eventPtr = 0;

        // 3) Sweep each slab [coords[i], coords[i+1])
        for (int i = 0; i < coords.size() - 1; i++) {
            double x = coords.get(i);
            double next = coords.get(i + 1);
            if (!(x + EPS < next)) continue; // skip zero-width slabs

            // Advance events at x: end before start already ensured by sort
            while (eventPtr < events.size() && almostEqual(events.get(eventPtr).x, x)) {
                Event e = events.get(eventPtr++);
                active[e.camId] = e.isStart; // start => true, end => false
            }

            // Collect active light intervals for this slab
            ArrayList<Interval> intervals = new ArrayList<>();
            for (int camId = 0; camId < active.length; camId++) {
                if (active[camId]) {
                    ClippedCam cc = clipped.get(camId);
                    // cc already clipped to required light; add directly
                    intervals.add(new Interval(cc.lMin, cc.lMax));
                }
            }

            // If no camera covers this slab at all => fail
            if (intervals.isEmpty()) return false;

            // Check if union of intervals covers [reqLMin, reqLMax]
            if (!coversFully(reqLMin, reqLMax, intervals)) {
                return false;
            }
        }

        // Also need to process events at the final coordinate (not required for slabs)
        return true;
    }

    private static boolean coversFully(double reqLMin, double reqLMax, List<Interval> intervals) {
        intervals.sort(Comparator.comparingDouble(a -> a.start));

        double coveredUntil = reqLMin;

        for (Interval in : intervals) {
            if (in.end <= coveredUntil + EPS) {
                continue; // interval is entirely before or touching current coverage
            }
            if (in.start > coveredUntil + EPS) {
                // gap found
                return false;
            }
            // extend coverage
            coveredUntil = Math.max(coveredUntil, in.end);
            if (coveredUntil >= reqLMax - EPS) {
                return true;
            }
        }
        return coveredUntil >= reqLMax - EPS;
    }

    private static ArrayList<Double> unique(ArrayList<Double> sorted) {
        ArrayList<Double> out = new ArrayList<>();
        Double prev = null;
        for (Double v : sorted) {
            if (prev == null || !almostEqual(prev, v)) {
                out.add(v);
                prev = v;
            }
        }
        return out;
    }

    private static boolean almostEqual(double a, double b) {
        return Math.abs(a - b) <= EPS;
    }

    // Optional quick sanity test
    public static void main(String[] args) {
        // Required: distance [0, 10], light [0, 10]
        double reqDMin = 0, reqDMax = 10;
        double reqLMin = 0, reqLMax = 10;

        List<Camera> cams = List.of(
                new Camera(0, 10, 0, 5),
                new Camera(0, 10, 5, 10)
        );

        System.out.println(camerasSuffice(reqDMin, reqDMax, reqLMin, reqLMax, cams)); // true

        List<Camera> cams2 = List.of(
                new Camera(0, 10, 0, 4),
                new Camera(0, 10, 6, 10)
        );
        System.out.println(camerasSuffice(reqDMin, reqDMax, reqLMin, reqLMax, cams2)); // false (gap 4..6)
    }
}
