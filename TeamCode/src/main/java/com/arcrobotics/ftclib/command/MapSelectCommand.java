package com.arcrobotics.ftclib.command;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.LogCatCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;

import java.util.Map;
import java.util.function.Supplier;

/**
 * Runs one of a selection of commands, either using a selector and a key to command mapping, or a
 * supplier that returns the command directly at runtime.  Does not actually schedule the selected
 * command - rather, the command is run through this command; this ensures that the command will
 * behave as expected if used as part of a CommandGroup.  Requires the requirements of all included
 * commands, again to ensure proper functioning when used in a CommandGroup.  If this is undesired,
 * consider using {@link ScheduleCommand}.
 *
 * <p>As this command contains multiple component commands within it, it is technically a command
 * group; the command instances that are passed to it cannot be added to any other groups, or
 * scheduled individually.
 *
 * <p>As a rule, CommandGroups require the union of the requirements of their component commands.
 */
public class MapSelectCommand<E> extends CommandBase {
    private final Map<E, ? extends Command> m_commands;
    private final Supplier<E> m_selector;
    private final Supplier<Command> m_toRun;
    private Command m_selectedCommand;

    /**
     * Creates a new selectcommand.
     *
     * @param commands the map of commands to choose from
     * @param selector the selector to determine which command to run
     */
    public MapSelectCommand(@NonNull Map<E, ? extends Command> commands, @NonNull Supplier<E> selector) {
        CommandGroupBase.registerGroupedCommands(commands.values().toArray(new Command[]{}));

        m_commands = commands;
        m_selector = selector;

        m_toRun = null;

        for (Command command : m_commands.values()) {
            m_requirements.addAll(command.getRequirements());
        }
    }

    /**
     * Creates a new selectcommand.
     *
     * @param toRun a supplier providing the command to run
     */
    public MapSelectCommand(@NonNull Supplier<Command> toRun) {
        m_commands = null;
        m_selector = null;

        m_toRun = toRun;
    }

    @Override
    public void initialize() {
        if (m_selector != null) {
            if (!m_commands.keySet().contains(m_selector.get())) {
                m_selectedCommand = new LogCatCommand(
                        "MapSelectCommand failure",
                        "MapSelectCommand selector value does not correspond to" + " any command!");
                return;
            }
            m_selectedCommand = m_commands.get(m_selector.get());
        } else {
            m_selectedCommand = m_toRun.get();
        }
        m_selectedCommand.initialize();
    }

    @Override
    public void execute() {
        m_selectedCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_selectedCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return m_selectedCommand.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        if (m_commands != null) {
            boolean runsWhenDisabled = true;
            for (Command command : m_commands.values()) {
                runsWhenDisabled &= command.runsWhenDisabled();
            }
            return runsWhenDisabled;
        } else {
            return m_toRun.get().runsWhenDisabled();
        }
    }
}