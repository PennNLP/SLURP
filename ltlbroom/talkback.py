"""Create natural language responses from command and error structures."""

# Copyright (C) 2014 Constantine Lignos
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


from semantics.lexical_constants import (GO_ACTION, PATROL_ACTION, ACTIVATE_ACTION)


class AbortError(Exception):
    """LTL generation crashed while interpreting the command."""
    pass


class LTLGenerationError(Exception):
    """Base class for LTL generation errors."""
    pass


class UnknownActionError(LTLGenerationError):
    """Action is not one the system can carry out."""
    pass


class BadArgumentError(LTLGenerationError):
    """Action has a bad argument."""
    pass


class BadConditionalError(LTLGenerationError):
    """Condition cannot be understood."""
    pass


class NoSuchLocationError(LTLGenerationError):
    """Location does not exist."""
    pass


class CommandResponse(object):
    """Structure for a response to a command."""

    def __init__(self, command, error=None):
        self.command = command
        self.error = error

    def __str__(self):
        if self.command:
            return self.command.readable()
        elif self.error:
            return str(self.error)
        else:
            return "Empty"

    def __repr__(self):
        return "<{}: {}>".format(self.__class__.__name__, str(self))


class ResponseInterpreter(object):
    """Interpret command responses into natural language."""

    CRASH = "Parsing failure when processing input."
    GOTIT = "I will {!r}"
    GOTIT_LONG = "I will {} {}"
    MISUNDERSTAND = "No verbs matched."
    CANNOT = "Robot cannot {!r}."
    CATCH_ALL = "Cannot explain error."
    NO_LOCATION = "No region {!r}."
    REGION = "region {!r}"
    PREP_REGION = "{} " + REGION
    CONDITION = "If {!r}"
    BAD_ARGUMENT = "Bad arguments for {!r}."
    BAD_CONDITION = "Bad condition for {!r}."

    def interpret(self, response):
        """Return a natural language explanation of a specgen response."""
        error = response.error
        if response.command:
            if not error:
                return self.command(response)
            else:
                if isinstance(error, UnknownActionError):
                    return self.cannot(response)
                elif isinstance(error, NoSuchLocationError):
                    return self.bad_location(response)
                elif isinstance(error, BadArgumentError):
                    return self.bad_argument(response)
                elif isinstance(error, BadConditionalError):
                    return self.bad_condition(response)
                else:
                    return self.CATCH_ALL
        elif error:
            if isinstance(error, AbortError):
                return self.CRASH
            else:
                return self.MISUNDERSTAND
        else:
            return self.MISUNDERSTAND

    def command(self, response):
        """Interpret a command."""
        command = response.command
        action = command.action

        prefix = None
        postfix = None
        if command.condition:
            prefix = self.CONDITION.format(command.condition.theme.name)
        elif action == ACTIVATE_ACTION and command.location:
            # Condition is a location, but command is not to go there
            # TODO: Commands other than activate may have the same behavior
            postfix = self.PREP_REGION.format("in", command.location.name)

        result = None
        if action == GO_ACTION:
            result = self.GOTIT_LONG.format(action,
                                            self.PREP_REGION.format("to", command.location.name))
        elif action == PATROL_ACTION:
            result = self.GOTIT_LONG.format(action, self.REGION.format(command.location.name))
        elif action == ACTIVATE_ACTION:
            result = self.GOTIT.format(command.theme.name)

        # Catch unhandled cases
        if not result:
            result = self.GOTIT.format(action)

        if prefix:
            result = "{}, {}".format(prefix, _lowercase_first(result))
        if postfix:
            result = "{} {}".format(result, postfix)

        result += "."

        return result

    def cannot(self, response):
        """Explain that the robot cannot carry out that action."""
        return self.CANNOT.format(response.command.action)

    def bad_location(self, response):
        """Explain that the location is not on the map."""
        return self.NO_LOCATION.format(response.error.message)

    def bad_argument(self, response):
        """Explain that an action has a bad argument."""
        return self.BAD_ARGUMENT.format(response.command.action)

    def bad_condition(self, response):
        """Explain that an action has a bad condition."""
        return self.BAD_CONDITION.format(response.error.message)


class FriendlyResponseInterpreter(ResponseInterpreter):
    """Human-friendly interpretation of command responses."""

    SORRY = "Sorry, "
    REWORD = " Try saying it differently."
    CRASH = SORRY + "something went wrong when I tried to understand that." + REWORD
    MISUNDERSTAND = SORRY + "I didn't understand what you said." + REWORD
    CANNOT = SORRY + "but I don't know how to {}."
    NO_LOCATION = SORRY + "but I don't know where {!r} is."
    CONDITION = "If I see a {}"
    CATCH_ALL = "That doesn't make sense to me, but I can't explain why."
    BAD_ARGUMENT = "I understood {!r}, but didn't get the rest."
    BAD_CONDITION = SORRY + "I don't understand what you mean by {!r}."
    REGION = "the room {!r}"
    PREP_REGION = "{} " + REGION


def _lowercase_first(astr):
    """Lowercase the first letter of a string, unless it's the word 'I'"""
    if not astr:
        return astr
    else:
        if len(astr) < 2 or astr[:2] != "I ":
            return astr[0].lower() + astr[1:]
        else:
            return astr
