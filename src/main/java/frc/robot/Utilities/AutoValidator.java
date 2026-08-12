package frc.robot.Utilities;

import java.io.File;
import java.io.IOException;
import java.nio.charset.MalformedInputException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Boot-time validator for PathPlanner autos.
 *
 * <p>PathPlannerAuto swallows load failures: if an auto references a path file that
 * isn't on the roboRIO, the constructor catches the exception, reports it once to the
 * DS console, and substitutes Commands.none(). The auto still shows up in the chooser
 * and "runs" -- it just does nothing. This class re-reads the deployed .auto JSON
 * itself and reports what's broken somewhere you'll actually see it.
 *
 * <p>Published keys (all under "Auto Check/"):
 * <ul>
 *   <li>All Autos OK      - boolean, false if ANY deployed auto is broken
 *   <li>Autos Scanned     - number of .auto files found
 *   <li>Problems          - multi-line summary of every problem found
 *   <li>Autos/&lt;name&gt;      - per-auto boolean
 *   <li>Selected          - name of the currently selected auto
 *   <li>Selected OK       - boolean for the currently selected auto  &lt;-- the one to put on the driver dashboard
 *   <li>Selected Problems - what's wrong with the current selection
 * </ul>
 * Broken autos also raise WPILib Alerts, which Elastic/Shuffleboard/AdvantageScope
 * display as persistent banners.
 */
public final class AutoValidator {
    /** Dashboard key prefix. */
    public static final String kDash = "Auto Check/";

    private static final String kAutoExt = ".auto";
    private static final String kPathExt = ".path";
    private static final String kTrajExt = ".traj";

    /** Chooser entry meaning "do nothing". Matches AutoBuilder's default label. */
    private static final String kNoneOption = "None";

    private static final ObjectMapper kMapper = new ObjectMapper();

    /** auto name -> list of problems (empty list == healthy). */
    private static final Map<String, List<String>> m_results = new LinkedHashMap<>();

    private static final List<Alert> m_bootAlerts = new ArrayList<>();
    private static final Alert m_selectedAlert =
            new Alert("Selected autonomous routine is broken", AlertType.kError);

    /** Auto name -> prebuilt command. Same objects the chooser hands back. */
    private static final Map<String, Command> m_commands = new LinkedHashMap<>();

    /** Set when the deploy tree itself is missing -- a failure with no auto to attach to. */
    private static boolean m_scanFailed = false;

    private static SendableChooser<String> m_chooser = null;
    private static String m_lastPublished = null;
    private static boolean m_selectedValid = true;

    private AutoValidator() {}

    // ---------------------------------------------------------------- scanning

    /**
     * Scans every deployed .auto file and publishes the results. Call this once, from
     * the RobotContainer constructor, AFTER configureNamedCommands() and after the
     * chooser has been published.
     *
     * @param chooserKey the SmartDashboard key the auto chooser was published under
     *                   (i.e. the string passed to SmartDashboard.putData)
     */
    public static void scan(String chooserKey) {
        m_results.clear();
        m_scanFailed = false;
        for (Alert a : m_bootAlerts) {
            a.set(false);
        }
        m_bootAlerts.clear();

        File autosDir = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
        File[] autoFiles = autosDir.listFiles((dir, name) -> name.endsWith(kAutoExt));

        if (autoFiles == null) {
            // Sticky: publishSummary() may run again after the chooser is built, and
            // an empty result map would otherwise republish "all clear".
            m_scanFailed = true;
            publishSummary();
            watchChooser(chooserKey);
            return;
        }

        Arrays.sort(autoFiles, Comparator.comparing(File::getName));

        // Exact-case name sets. Deliberately NOT File.exists(): macOS/Windows are
        // case-insensitive, the roboRIO's ext4 is not, so exists() would pass in
        // simulation and fail on the real robot.
        Set<String> pathNames = listNames(new File(Filesystem.getDeployDirectory(), "pathplanner/paths"), kPathExt);
        Set<String> choreoNames = listNames(new File(Filesystem.getDeployDirectory(), "choreo"), kTrajExt);

        for (File f : autoFiles) {
            String autoName = stripExt(f.getName(), kAutoExt);
            m_results.put(autoName, validate(f, pathNames, choreoNames));
        }

        publishSummary();
        watchChooser(chooserKey);
    }

    private static List<String> validate(File autoFile, Set<String> pathNames, Set<String> choreoNames) {
        List<String> problems = new ArrayList<>();

        String autoFileProblem = describeFileProblem(autoFile);
        if (autoFileProblem != null) {
            problems.add("auto file " + autoFileProblem);
            return problems;
        }

        File pathsDir = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");

        try {
            JsonNode root = kMapper.readTree(autoFile);
            boolean choreo = root.path("choreoAuto").asBoolean(false);
            JsonNode commandRoot = root.path("command");

            if (!commandRoot.isObject()) {
                // A deliberate "do nothing" auto still has a command tree with an empty
                // commands array, so this only fires on a genuinely malformed or
                // wrong-schema file.
                problems.add("auto file has no command tree (empty or wrong-version file)");
                return problems;
            }

            Set<String> referencedPaths = new LinkedHashSet<>();
            Set<String> referencedCommands = new LinkedHashSet<>();
            collect(commandRoot, referencedPaths, referencedCommands);

            for (String p : referencedPaths) {
                if (choreo) {
                    // Choreo trajectories may be referenced with a split index, e.g.
                    // "FourPiece.2", which all live in the single FourPiece.traj file.
                    String base = stripSplitIndex(p);
                    if (!choreoNames.contains(p) && !choreoNames.contains(base)) {
                        problems.add("missing Choreo trajectory \"" + base + kTrajExt + "\"");
                        continue;
                    }
                    String loadError = tryLoadChoreo(base);
                    if (loadError != null) {
                        problems.add("Choreo trajectory \"" + base + kTrajExt + "\" won't load: " + loadError);
                    }
                    continue;
                }

                // 1. Does the file exist, with exactly this capitalization?
                if (!pathNames.contains(p)) {
                    problems.add("missing path file \"" + p + kPathExt + "\"");
                    continue;
                }

                // 2. Is the file's content sane? This produces a human-readable cause
                //    (empty file, merge conflict markers, BOM) instead of a parser dump.
                String fileProblem = describeFileProblem(new File(pathsDir, p + kPathExt));
                if (fileProblem != null) {
                    problems.add("path file \"" + p + kPathExt + "\" " + fileProblem);
                    continue;
                }

                // 3. Authoritative check: run PathPlanner's own loader. If this throws,
                //    the auto is broken no matter how healthy the file looks to us --
                //    version mismatches and schema changes surface here and nowhere else.
                String loadError = tryLoadPath(p);
                if (loadError != null) {
                    problems.add("path \"" + p + "\" won't load: " + loadError);
                }
            }

            for (String c : referencedCommands) {
                if (!NamedCommands.hasCommand(c)) {
                    problems.add("unregistered NamedCommand \"" + c + "\"");
                }
            }
        } catch (IOException e) {
            // Jackson's messages are multi-line and carry a source-location footer,
            // which turns a dashboard string into a mess. First line is the useful part.
            problems.add("auto file unreadable or malformed (" + firstLine(e.getMessage()) + ")");
        }
        return problems;
    }

    /**
     * Diagnoses why a JSON file would fail to load, in plain language. Returns null if
     * the file is fine.
     *
     * <p>Note the BOM case: Jackson silently tolerates a UTF-8 byte order mark, but the
     * json-simple parser bundled in PathPlannerLib does not. A file that looks valid to
     * every tool on your laptop can still hard-fail on the robot.
     */
    private static String describeFileProblem(File file) {
        String text;
        try {
            text = Files.readString(file.toPath(), StandardCharsets.UTF_8);
        } catch (MalformedInputException e) {
            return "is not valid UTF-8 text (binary or wrong encoding)";
        } catch (IOException e) {
            return "could not be read (" + firstLine(e.getMessage()) + ")";
        }

        if (!text.isEmpty() && text.charAt(0) == '\uFEFF') {
            return "starts with a UTF-8 BOM, which PathPlanner's JSON parser rejects";
        }
        if (text.isBlank()) {
            return "is empty (0 bytes / whitespace only)";
        }
        if (text.contains("<<<<<<<") || text.contains(">>>>>>>") || text.contains("=======\n")) {
            return "contains unresolved git merge conflict markers";
        }
        try {
            kMapper.readTree(text);
        } catch (Exception e) {
            return "is not valid JSON (" + firstLine(e.getMessage()) + ")";
        }
        return null;
    }

    /** Runs PathPlanner's real path loader. Returns null on success, else the reason. */
    private static String tryLoadPath(String pathName) {
        try {
            PathPlannerPath.fromPathFile(pathName);
            return null;
        } catch (Exception e) {
            return describeThrowable(e);
        }
    }

    /** Runs PathPlanner's real Choreo loader. Returns null on success, else the reason. */
    private static String tryLoadChoreo(String trajectoryName) {
        try {
            PathPlannerPath.fromChoreoTrajectory(trajectoryName);
            return null;
        } catch (Exception e) {
            return describeThrowable(e);
        }
    }

    private static String describeThrowable(Throwable t) {
        String message = t.getMessage();
        return t.getClass().getSimpleName()
                + (message == null || message.isBlank() ? "" : ": " + firstLine(message));
    }

    /**
     * Depth-first walk of the auto's command tree. Every composition type
     * (sequential / parallel / race / deadline) stores its children under
     * data.commands, and wrapper types store a single child under data.command,
     * so recursing on both covers the whole schema without hardcoding type names.
     */
    private static void collect(JsonNode cmd, Set<String> paths, Set<String> named) {
        if (cmd == null || !cmd.isObject()) {
            return;
        }

        String type = cmd.path("type").asText("");
        JsonNode data = cmd.path("data");

        if ("path".equals(type)) {
            String p = data.path("pathName").asText("");
            if (!p.isEmpty()) {
                paths.add(p);
            }
        } else if ("named".equals(type)) {
            String n = data.path("name").asText("");
            if (!n.isEmpty()) {
                named.add(n);
            }
        }

        JsonNode children = data.path("commands");
        if (children.isArray()) {
            for (JsonNode child : children) {
                collect(child, paths, named);
            }
        }

        JsonNode single = data.path("command");
        if (single.isObject()) {
            collect(single, paths, named);
        }
    }

    // -------------------------------------------------------------- publishing

    private static void publishSummary() {
        boolean allOk = !m_scanFailed;
        StringBuilder summary = new StringBuilder();

        if (m_scanFailed) {
            String line = "No deploy/pathplanner/autos directory found on the robot -- did the deploy finish?";
            summary.append(line).append('\n');
            fail(line);
        }

        for (Map.Entry<String, List<String>> e : m_results.entrySet()) {
            String name = e.getKey();
            List<String> problems = e.getValue();
            SmartDashboard.putBoolean(kDash + "Autos/" + name, problems.isEmpty());

            if (!problems.isEmpty()) {
                allOk = false;
                String line = name + " -> " + String.join("; ", problems);
                summary.append(line).append('\n');
                fail(line);
            }
        }

        SmartDashboard.putBoolean(kDash + "All Autos OK", allOk);
        SmartDashboard.putNumber(kDash + "Autos Scanned", m_results.size());
        SmartDashboard.putString(kDash + "Problems",
                summary.length() == 0 ? "none" : summary.toString().trim());
    }

    private static void fail(String message) {
        DriverStation.reportError("[AutoValidator] " + message, false);
        Alert alert = new Alert("[Auto] " + message, AlertType.kError);
        alert.set(true);
        m_bootAlerts.add(alert);
    }

    private static void watchChooser(String chooserKey) {
        m_lastPublished = null;
        periodic();
    }

    /**
     * Builds the auto chooser, validates every auto, and publishes it under
     * {@code dashboardKey}. Use this INSTEAD of AutoBuilder.buildAutoChooser().
     *
     * <p>The chooser holds auto NAMES rather than Commands, which is the whole point:
     * the selection tracker can then ask the chooser object directly what's selected
     * instead of trying to reverse-engineer it from NetworkTables. The commands are
     * still built once at boot, so nothing is loaded on the fly at auto start.
     *
     * <p>Must be called after NamedCommands registration and after AutoBuilder.configure().
     */
    public static SendableChooser<String> buildChooser(String dashboardKey) {
        scan(dashboardKey);

        m_commands.clear();
        SendableChooser<String> chooser = new SendableChooser<>();
        chooser.setDefaultOption(kNoneOption, kNoneOption);
        m_commands.put(kNoneOption, Commands.none());

        for (String autoName : m_results.keySet()) {
            Command command;
            try {
                command = new PathPlannerAuto(autoName);
            } catch (Exception e) {
                // Some PathPlannerLib versions let load failures escape the constructor
                // instead of swallowing them. Either way the auto is unusable, and
                // either way we record it rather than letting it take the robot down.
                m_results.get(autoName).add("PathPlannerAuto failed to construct: " + describeThrowable(e));
                command = Commands.none();
            }
            m_commands.put(autoName, command);
            chooser.addOption(autoName, autoName);
        }

        // Re-publish: construction may have added problems the file scan couldn't see.
        publishSummary();

        m_chooser = chooser;
        m_lastPublished = null;
        SmartDashboard.putData(dashboardKey, chooser);
        periodic();
        return chooser;
    }

    /** The command for the current selection. Use this in getAutonomousCommand(). */
    public static Command getSelectedCommand() {
        String name = getSelectedName();
        Command command = m_commands.get(name);

        if (command == null) {
            DriverStation.reportError("[AutoValidator] no command built for auto \"" + name + "\"", false);
            return Commands.none();
        }
        if (!getProblems(name).isEmpty()) {
            DriverStation.reportError("[AutoValidator] RUNNING A BROKEN AUTO: \"" + name + "\" -> "
                    + String.join("; ", getProblems(name)), false);
        }
        return command;
    }

    /** Name of the currently selected auto, straight from the chooser object. */
    public static String getSelectedName() {
        if (m_chooser == null) {
            return kNoneOption;
        }
        String selected = m_chooser.getSelected();
        return selected == null ? kNoneOption : selected;
    }

    /** Call every loop from Robot.robotPeriodic(). Cheap: only republishes on change. */
    public static void periodic() {
        if (m_chooser == null) {
            return;
        }

        String selected = getSelectedName();
        if (selected.equals(m_lastPublished)) {
            return;
        }
        m_lastPublished = selected;

        SmartDashboard.putString(kDash + "Selected", selected);

        if (kNoneOption.equals(selected)) {
            m_selectedValid = true;
            m_selectedAlert.set(false);
            SmartDashboard.putBoolean(kDash + "Selected OK", true);
            SmartDashboard.putString(kDash + "Selected Problems", "no auto selected");
            return;
        }

        List<String> problems = m_results.get(selected);
        String detail;
        if (problems == null) {
            detail = "\"" + selected + "\" is not a deployed .auto file";
        } else if (problems.isEmpty()) {
            detail = "none";
        } else {
            detail = String.join("; ", problems);
        }

        m_selectedValid = (problems != null && problems.isEmpty());
        SmartDashboard.putBoolean(kDash + "Selected OK", m_selectedValid);
        SmartDashboard.putString(kDash + "Selected Problems", detail);

        if (!m_selectedValid) {
            m_selectedAlert.setText("Selected auto \"" + selected + "\": " + detail);
            m_selectedAlert.set(true);
            DriverStation.reportWarning("[AutoValidator] selected auto \"" + selected + "\": " + detail, false);
        } else {
            m_selectedAlert.set(false);
        }
    }

    // ------------------------------------------------------------------ queries

    /** True if the currently selected auto loaded cleanly. */
    public static boolean isSelectedValid() {
        return m_selectedValid;
    }

    /** True if every deployed auto loaded cleanly. */
    public static boolean allValid() {
        for (List<String> problems : m_results.values()) {
            if (!problems.isEmpty()) {
                return false;
            }
        }
        return true;
    }

    /** Problems for a specific auto; empty if healthy or unknown. */
    public static List<String> getProblems(String autoName) {
        return m_results.getOrDefault(autoName, List.of());
    }

    // ------------------------------------------------------------------ helpers

    private static Set<String> listNames(File dir, String ext) {
        Set<String> names = new HashSet<>();
        File[] files = dir.listFiles((d, n) -> n.endsWith(ext));
        if (files != null) {
            for (File f : files) {
                names.add(stripExt(f.getName(), ext));
            }
        }
        return names;
    }

    private static String stripExt(String fileName, String ext) {
        return fileName.substring(0, fileName.length() - ext.length());
    }

    private static String firstLine(String message) {
        if (message == null) {
            return "unknown error";
        }
        int newline = message.indexOf('\n');
        String line = (newline < 0 ? message : message.substring(0, newline)).trim();
        return line.length() > 120 ? line.substring(0, 117) + "..." : line;
    }

    /** "FourPiece.2" -> "FourPiece"; leaves names without a numeric suffix alone. */
    private static String stripSplitIndex(String name) {
        int dot = name.lastIndexOf('.');
        if (dot <= 0 || dot == name.length() - 1) {
            return name;
        }
        for (int i = dot + 1; i < name.length(); i++) {
            if (!Character.isDigit(name.charAt(i))) {
                return name;
            }
        }
        return name.substring(0, dot);
    }
}