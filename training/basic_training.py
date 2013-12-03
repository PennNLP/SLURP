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

from semantics.new_structures import Command, ObjectEntity, Location
from training.semantics_handler import SemanticsHandler
from pipelinehost import PipelineClient
from semantics.tree import Tree

OK = "OK"
NOT_OK = "NOT_OK"

class Go(object):
    def __init__(self,exercise="go_simple"):
        self.handler = SemanticsHandler()
        self.next = "next"
        self.prompt = "prompt"
        self.response = "response"
        self.agent_object = "object"
        self.agents = [self.agent_object]
        self.action = "Action"
        self.get_input = raw_input
        self.correct_commands = "correct_commands"
        self.default_action = "default_action"
        self.regions = {   "office" : "office" ,
                        "conservatory" : "conservatory",
                        "cellar" : "cellar"
                    }    
        self.moderate_regions = {   "office" : "office" ,
                        "conservatory" : "conservatory",
                    }    
        self.advanced_regions = {   "office" : "office" ,
                        "conservatory" : "conservatory",
                        "cellar" : "cellar"
                    }   
        self.ccs = { "and" : "and"}
        self.current_exercise = exercise
        self.train_functions = {"go_simple" : self.go_simple, "go_moderate" : self.go_moderate, "go_advanced" : self.go_advanced}
        self.train_dict = {"go_simple" : {self.prompt : "Tell Junior to go to the %s" % self.regions["office"],
                               self.default_action : "go",
                               self.correct_commands : [self.command_from_dict({'Source': '',
                                                  'Theme': '',
                                                  'Condition': '',
                                                  'Patient': '',
                                                  'Location': {'Location': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1},
                                                                            'Name': self.regions["office"],
                                                                            'Description': []}},
                                                  'Action': 'go',
                                                  'Destination': '',
                                                  'Negation': False,
                                                  'Agent': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1},
                                                                       'Name': None, 'Description': []}}})],
                               self.response: "Good work! Junior will go.",
                               self.next : "go_moderate"
                               },
                "go_moderate" : {self.prompt : "Tell Junior to go to the office and the conservatory.",
                               self.default_action : "go",
                               self.correct_commands : [self.command_from_dict({'Source': '',
                                                  'Theme': '',
                                                  'Condition': '',
                                                  'Patient': '',
                                                  'Location': {'Location': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1},
                                                                            'Name': w,
                                                                            'Description': []}},
                                                  'Action': 'go',
                                                  'Destination': '',
                                                  'Negation': False,
                                                  'Agent': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1},
                                                                       'Name': None, 'Description': []}}}) for w in self.moderate_regions],
                               self.response: "Good work! Junior will go.",
                               self.next: "go_simple"
                               },    
                "go_advanced" : {self.prompt : "Tell Junior to go to the office, the conservatory, and the cellar.",
                             self.default_action : "go",
                               self.correct_commands : [self.command_from_dict({'Source': '',
                                                  'Theme': '',
                                                  'Condition': '',
                                                  'Patient': '',
                                                  'Location': {'Location': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1},
                                                                            'Name': w,
                                                                            'Description': []}},
                                                   'Action': 'go',
                                                   'Destination': '',
                                                   'Negation': False,
                                                   'Agent': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1},
                                                                        'Name': None, 'Description': []}}}) for w in self.advanced_regions],
                               self.response: "Good work! Junior will go.",
                               self.next: "go_simple"
                               }          
                }

    def go_exercise(self,exercise,sentence):
        if not sentence:
            sentence = self.get_input(exercise[self.prompt]+":\n")
        commands = self.handler.get_semantic_structures(sentence)  
        answers = exercise[self.correct_commands]          
        success =  all(command in answers for command in commands)
        if len(commands) < len(answers):
            for answer in answers:
                if not answer in commands:
                    response = "You forgot to tell Junior to " + answer.action
                    if answer.location:
                        response += " to the " + answer.location.name
            response = NOT_OK + response            
        elif success:
            response = exercise[self.response] 
            response = OK+response
        else:
            #Return the diff response for the first failed command only
            for command in commands:
                if not command in answers: 
                    diffs = [self.get_command_diff(command,w) for w in answers]
                    response = self.get_diff_response(diffs)
                    response = NOT_OK + response
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
                    
    def get_diff_response(self,diffs):
        unique_diffs = [(w,diff[w]) for diff in diffs for w in diff if not diff[w]]
        return str(unique_diffs)
                
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
        return False
        
    def get_current_prompt(self):
        return self.train_dict[self.current_exercise][self.prompt]
    
    def command_from_dict(self,d):
        #agents and patients
        if d["Patient"] != "":
            patient_object = d["Patient"]["Object"]
            patient = ObjectEntity.from_dict(patient_object)
        else:
            patient = None
        if d["Agent"] != "":
            agent_object = d["Agent"]["Object"]
            agent = ObjectEntity.from_dict(agent_object)
        else:
            agent = None
        #Locations
        if d["Location"] != "":
            location_location = d["Location"]["Location"]
            location = Location.from_dict(location_location)
        else:
            location = None
        if d["Destination"] != "":
            destination_location = d["Destination"]["Location"]
            destination = Location.from_dict(destination_location)
        else:
            destination = None    
        if d["Source"] != "":
            source_location = d["Source"]["Location"]
            source = Location.from_dict(source_location)
        else:
            source = None 
        #Themes  
        if d["Theme"] == "":
            theme = None
        #Conditions
        if d["Condition"] != "":
            if "Command" in d["Condition"]:
                condition_command = d["Condition"]["Command"]
                condition = self.command_from_dict(condition_command)
            elif "Assertion" in d["Condition"]:
                #Not training on assertions, skip for now
                condition = None           
            else:
                condition = None   
        else:
            condition = None     
        return Command(agent,theme,patient,location,source,destination,d["Action"],condition,d["Negation"])

        
        