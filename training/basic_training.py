"""
A basic training instance for SLURP semantic command interpretation.
"""

# Copyright (C) 2013 Taylor Turpen
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

from semantics.new_structures import Command
from training.semantics_handler import SemanticsHandler
from training.training_dictionaries import TrainingDictionary
from pipelinehost import PipelineClient
from semantics.tree import Tree
from copy import deepcopy


OK = "OK"
NOT_OK = "NOT_OK"
       
class Go(object):
    def __init__(self,exercise="go_simple"):
        self.handler = SemanticsHandler()
        td = TrainingDictionary()
        self.train_dict = td.train_dict
        self.prompt = td.prompt
        self.canonical_commands = td.canonical_commands
        self.alternate_commands = td.alternate_commands
        self.response = td.response
        self.next = td.next
        self.current_exercise = exercise
        self.train_functions = {"go_simple" : self.go_simple,
                                "go_moderate" : self.go_moderate,
                                "go_advanced" : self.go_advanced,
                                "defuse_simple" : self.defuse_simple,
                                "defuse_moderate" : self.defuse_moderate,
                                "conditional_simple" : self.conditional_simple}

    def respond_missing_answer(self,commands,answers,alternates):
        """If there is a command in answers/alternates, pop answers/alternates until we run out of commands"""
        # TODO: Answers and alternates should have some hierarchy like answer->alternate->fail       
        if commands and commands[0] in answers:
            for answer in answers:
                if not answer in commands:
                    response = "You forgot to tell Junior to " + answer.action
                    if answer.location:
                        response += " to the " + answer.location.name                     
        elif commands and commands[0] in alternates:
            for alternate in alternates:
                if not alternate in commands:
                    response = "You forgot to tell Junior to " + alternate.action
                    if alternate.location:
                        response += " to the " + alternate.location.name 
        else:
            response = "Too few commands. Try using lists or conjunctions."
        response += "."
        response = NOT_OK + response
        return response            

        
    def go_exercise(self,exercise,sentence):    
        """The canonical exercise method from the specific entries in the training dictionary.
    
            @input exercise, the specific exercise from the training dictionary
            @input sentence, the sentence entered by the user
        """       
        if not sentence:
            sentence = self.get_input(exercise[self.prompt]+":\n")
        #Get the semantic frame from the semantic handler
        commands = self.handler.get_semantic_structures(sentence) 
        if not commands:
            return NOT_OK + "I was not able to undersand what you meant, please try again." 
        answers = exercise[self.canonical_commands] 
        alternates = exercise[self.alternate_commands]
        success =  all(command in answers or command in alternates for command in commands)
        if len(commands) < len(answers):
            response = self.respond_missing_answer(commands,answers,alternates)
        elif success:
            response = exercise[self.response] 
            response = OK+response
        else:
            #Return the diff response for the first failed command only
            for command in commands:
                if not command in answers or not command in alternates:
                    if not command in answers: 
                        diffs = [self.get_command_diff(command,w) for w in answers]
                        response = self.get_diff_response(diffs)
                        response = NOT_OK + response
                        return response
                    diffs = [self.get_command_diff(command,w) for w in alternates]
                    response = self.get_diff_response(diffs)
                    response = NOT_OK + response
                    return response
        return response
    
    def go_simple(self,sentence=None):
        exercise = self.train_dict["go_simple"]
        return self.go_exercise(exercise, sentence)
        
    def go_moderate(self,sentence=None):
        exercise = self.train_dict["go_moderate"]
        return self.go_exercise(exercise, sentence)
    
    def go_advanced(self,sentence=None):
        exercise = self.train_dict["go_advanced"]
        return self.go_exercise(exercise, sentence)
    
    def defuse_simple(self,sentence=None):
        exercise = self.train_dict["defuse_simple"]
        return self.go_exercise(exercise, sentence)
    
    def defuse_moderate(self,sentence=None):
        exercise = self.train_dict["defuse_moderate"]
        return self.go_exercise(exercise, sentence)
    
    def conditional_simple(self,sentence=None):
        exercise = self.train_dict["conditional_simple"]
        return self.go_exercise(exercise, sentence)
                    
    def get_diff_response(self,diffs):
        base = "I am afraid you entered the wrong "
        tail = "Please try again."
        unique_diffs = list(set([(w,diff[w]) for diff in diffs for w in diff if not diff[w]]))
        unique_wrong_members = [w[0] for w in unique_diffs]
        if len(unique_wrong_members) > 1:            
            return base + ", ".join(unique_wrong_members[:-1]) + " and " + str(unique_wrong_members[-1]) 
        return base + str(unique_wrong_members[0]) + ". " + tail 
                
    def get_command_diff(self,this,that):
        d = {"instance": isinstance(that,Command),
             "agent":  this.agent == that.agent,             
             "action":  this.action == that.action,
             "theme": this.theme == that.theme,             
             "patient": this.patient == that.patient,
             "location": this.location == that.location,
             "source": this.source == that.source,
             "destination": this.destination == that.destination,
             "condition": this.condition == that.condition,
             "negation": this.negation == that.negation
             }
        return d
                    
                
    def run_current_exercise(self,sentence):
        if self.validate_sentence(sentence):
            return self.train_functions[self.current_exercise](sentence)
        
    def get_next_exercise(self):
        return self.train_dict[self.current_exercise][self.next]
            
    def validate_sentence(self,sentence):
        if sentence[-1] == ".": return True
        return NOT_OK + "Please make sure that your command ends in a period(.)."
        
    def get_current_prompt(self):
        return self.train_dict[self.current_exercise][self.prompt]   
 
        
        