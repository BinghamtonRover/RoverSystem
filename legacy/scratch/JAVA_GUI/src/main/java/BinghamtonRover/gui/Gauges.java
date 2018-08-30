package BinghamtonRover.gui;

import eu.hansolo.medusa.*;
import javafx.animation.AnimationTimer;
import javafx.geometry.Orientation;
import javafx.scene.paint.Color;

import java.util.Random;

public final class Gauges {

    private static final Color SILVERBLUE = Color.rgb(0x5C,0x8E,0xB5);
    private static final Color ONYX       = Color.rgb(0x38,0x38,0x38);
    private static final Color VERMILLION = Color.rgb(0xE2,0x40,0x3B);
    private static final Color ALMOND     = Color.rgb(0xEF,0xDC,0xCC);
    private static final Color PLATINUM   = Color.rgb(0xE4,0xE1,0xDF);

    private static final Random RND = new Random();
    private static long         lastTimerCall;

    public static Gauge PRESSURE_GAUGE = GaugeBuilder.create()
            .foregroundBaseColor(PLATINUM)
            .barColor(VERMILLION)
            .skinType(Gauge.SkinType.LINEAR)
            .orientation(Orientation.HORIZONTAL)
            .prefSize(260, 200)
            .title("Air Pressure")
            .unit("PSI")
            .startAngle(270)     //Origin 270
            .angleRange(180)    //Origin 270
            .minValue(0)
            .maxValue(2000)
            .averageVisible(true)
            .averagingEnabled(true)
            .averagingPeriod(15)
            .tickLabelLocation(TickLabelLocation.OUTSIDE)
            .tickLabelOrientation(TickLabelOrientation.ORTHOGONAL)
            .tickMarkColor(ALMOND)
            .onlyFirstAndLastTickLabelVisible(true)
            .scaleDirection(Gauge.ScaleDirection.COUNTER_CLOCKWISE)
            .minorTickMarksVisible(false)
            .majorTickMarkType(TickMarkType.BOX)
            .valueVisible(true)
            .knobType(Gauge.KnobType.METAL)
            .interactive(true)
            .onButtonPressed(o -> System.out.println("psur knob Button pressed"))
            .onButtonReleased(o -> System.out.println("psur knob Button released"))
            .needleShape(Gauge.NeedleShape.FLAT)
            .needleColor(Color.CADETBLUE)
            .sectionsVisible(true)
            .animated(true)
            .animationDuration(100)
            .build();

    public static Gauge TEMPERATURE_GAUGE = GaugeBuilder.create()
            .foregroundBaseColor(PLATINUM)
            .skinType(Gauge.SkinType.LINEAR)
            .orientation(Orientation.HORIZONTAL)
            .barColor(VERMILLION)
            .prefSize(260, 200)
            .title("Temperature")
            .unit("°C")
            .startAngle(270)     //Origin 270
            .angleRange(180)    //Origin 270
            .minValue(0)
            .maxValue(500)
            .averageVisible(true)
            .averagingEnabled(true)
            .averagingPeriod(15)
            .averageColor(Color.ORANGERED)
            .tickLabelLocation(TickLabelLocation.OUTSIDE)
            .tickLabelOrientation(TickLabelOrientation.ORTHOGONAL)
            .minorTickMarksVisible(false)
            .majorTickMarkType(TickMarkType.BOX)
            .valueVisible(true)
            .knobType(Gauge.KnobType.METAL)
            .interactive(true)
            .onButtonPressed(o -> System.out.println("temp knob Button pressed"))
            .onButtonReleased(o -> System.out.println("temp knob Button released"))
            .needleShape(Gauge.NeedleShape.FLAT)
            .needleColor(Color.ORANGERED)
            .sectionsVisible(true)
            .sections(new Section(0, 273, Color.rgb(100, 168, 199, 0.9)),
                    new Section(373, 500, Color.rgb(230, 55, 69, 0.9)))
            .animated(true)
            .animationDuration(100)
            .build();

    public static Gauge HUMIDITY_GAUGE= GaugeBuilder.create()
            .foregroundBaseColor(PLATINUM)
            .barColor(VERMILLION)
            .skinType(Gauge.SkinType.LINEAR)
            .orientation(Orientation.HORIZONTAL)
            .prefSize(260, 200)
            .title("Humidity")
            .unit("g/m^3")
            .startAngle(270)     //Origin 270
            .angleRange(180)    //Origin 270
            .minValue(0)
            .maxValue(100)
            .averageVisible(true)
            .averagingEnabled(true)
            .averagingPeriod(15)
            .tickLabelLocation(TickLabelLocation.OUTSIDE)
            .tickLabelOrientation(TickLabelOrientation.ORTHOGONAL)
            .tickMarkColor(ALMOND)
            .onlyFirstAndLastTickLabelVisible(true)
            .scaleDirection(Gauge.ScaleDirection.COUNTER_CLOCKWISE)
            .minorTickMarksVisible(false)
            .majorTickMarkType(TickMarkType.BOX)
            .valueVisible(true)
            .knobType(Gauge.KnobType.METAL)
            .interactive(true)
            .onButtonPressed(o -> System.out.println("psur knob Button pressed"))
            .onButtonReleased(o -> System.out.println("psur knob Button released"))
            .needleShape(Gauge.NeedleShape.FLAT)
            .needleColor(Color.CADETBLUE)
            .sectionsVisible(true)
            .animated(true)
            .animationDuration(100)
            .build();

    public static Gauge PRESSURE_GRAPH = GaugeBuilder.create()
            .skinType(Gauge.SkinType.TILE_SPARK_LINE)
            .minValue(PRESSURE_GAUGE.getMinValue())
            .maxValue(PRESSURE_GAUGE.getMaxValue())
            .barColor(SILVERBLUE)
            .prefSize(125, 125)
            .averageVisible(true)
            .averagingEnabled(true)
            .averagingPeriod(20)
            .animated(true)
            .build();

    public static Clock CLOCK = ClockBuilder.create()
            .skinType(Clock.ClockSkinType.DIGITAL)
            .running(true)
            .textColor(ALMOND)
            .dateColor(ALMOND)
            .build();

    private static AnimationTimer TIMER  = new AnimationTimer() {

        @Override public void handle(long now) {
            if (now > lastTimerCall + 1_000_000_000) {
                PRESSURE_GAUGE.setValue(RND.nextGaussian() * 34 + 600);
                TEMPERATURE_GAUGE.setValue(RND.nextGaussian() * 25 + 205);
                HUMIDITY_GAUGE.setValue(RND.nextGaussian() * 10 + 80);
                PRESSURE_GRAPH.setValue(PRESSURE_GAUGE.getValue());
                lastTimerCall = now;
                lastTimerCall = System.nanoTime();

            }
        }
    };

    public static void StartAnimation(){
        TIMER.start();
    }

}
