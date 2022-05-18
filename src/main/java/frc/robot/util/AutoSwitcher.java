package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class AutoSwitcher {
    public enum Auto {
        ONEBALLAUTO, TWOBALLAUTO, THREEBALLAUTO
    }

    private static final String tableKey = "AutoSwitcher";
    private static SendableChooser<Auto> autoChooser;

    private static ExampleSubsystem placeholder;

    private String key;
    private Auto defaultAuto;
    private Auto previousAuto = defaultAuto;
    public Command finalAuto;
    private List<Consumer<Auto>> switcher;

    /**
     * Create a new TunableNumber
     * 
     * @param dashboardKey Key on dashboard
     */
    public AutoSwitcher() {
        this.key = tableKey + "/" + "AutoSwitcher";
    }

    /**
     * Create a new TunableNumber with the default value
     * 
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public AutoSwitcher(Auto defaultAuto, ExampleSubsystem subsystem) {
        placeholder = subsystem;
        setDefault(defaultAuto);
        autoChooser = new SendableChooser<Auto>();
        autoChooser.setDefaultOption("One Ball", Auto.ONEBALLAUTO);
        autoChooser.addOption("Two Ball", Auto.TWOBALLAUTO);
        autoChooser.addOption("Three", Auto.THREEBALLAUTO);
        addChangeListener((auto) -> finalAuto = autoTranslator(auto));
    }

    /**
     * Get the default value for the number that has been set
     * 
     * @return The default value
     */
    public Auto getDefault() {
        return defaultAuto;
    }

    public Command autoTranslator(Auto auto) {
        Auto autonomous = auto;
        switch (autonomous) {
            case ONEBALLAUTO:
                return new ExampleCommand(placeholder);
            case TWOBALLAUTO:
                return new ExampleCommand(placeholder);
            case THREEBALLAUTO:
            default:
                return new ExampleCommand(placeholder);

        }
    }

    /**
     * Set the default value of the number
     * 
     * @param defaultValue The default value
     */
    public void setDefault(Auto defaultAuto) {
        this.defaultAuto = defaultAuto;
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode
     * 
     * @return The current value
     */
    public Auto get() {
        Auto selectedAuto = autoChooser.getSelected();

        if (selectedAuto != previousAuto) {
            previousAuto = selectedAuto;
            notify(selectedAuto);
        }

        return selectedAuto;
    }

    public void addChangeListener(Consumer<Auto> listener) {
        if (listener == null) {
            return;
        }

        if (switcher == null) {
            switcher = new ArrayList<>();
        }

        switcher.add(listener);
    }

    private void notify(Auto auto) {
        if (switcher == null) {
            return;
        }
        for (var switcher : switcher) {
            switcher.accept(auto);
        }
    }
}
