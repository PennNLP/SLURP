"""
A basic training dictionary and interpretation.
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
from semantics.lexical_constants import ACTION_ALIASES

class TrainingDictionary(object):
    def __init__(self):
        self.canonical_robot_name = "junior"
        self.alternate_robot_name = None
        self.next = "next"
        self.prompt = "prompt"
        self.response = "response"
        self.agent_object = "object"
        self.canonical_sentence = "canonical"
        self.agents = [self.agent_object]
        self.action = "Action"
        self.get_input = raw_input
        #Canonical commands are the minimal set of commands required for the exercise.
        self.canonical_commands = "canonical_commands"
        #Alternate commands are the canonical commands with some values replaced.
        self.alternate_commands = "alternate_commands"
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
        self.train_dict = { "login" : {self.prompt : "Please enter your uuid.",
                                       self.next : "go_simple"},                           
                "go_simple" : {self.prompt : "Junior should be in the office. Give him a command that will make him go there.",
                               self.default_action : "go",
                               self.canonical_commands : [self.command_from_dict({'Source': '',
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
                                                                       'Name': self.canonical_robot_name, 'Description': []}}})],
                                self.alternate_commands : [self.command_from_dict({'Source': '',
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
                                                                       'Name': self.alternate_robot_name, 'Description': []}}})],
                               self.response: "Good work! Junior will go.",
                               self.next : "go_moderate"
                               },
                "go_moderate" : {self.prompt : "Junior needs to go to the conservatory and the office. Give him a one-sentence command that will make him go to both places.",
                               self.default_action : "go",
                               self.canonical_commands : [self.command_from_dict({'Source': '',
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
                                                                       'Name': self.canonical_robot_name, 'Description': []}}}) for w in self.moderate_regions],
                               self.alternate_commands : [self.command_from_dict({'Source': '',
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
                                                                       'Name': self.alternate_robot_name, 'Description': []}}}) for w in self.moderate_regions],                                 
                               self.response: "Good work! Junior will go.",
                               self.next: "go_advanced"
                               },    
                "go_advanced" : {self.prompt : "Tell Junior to go to the office, the conservatory, and the cellar. Give him a one-sentence command that will make him go to all three places. Remember, Junior is a stickler for punctuation! When using a list, make sure to put a comma before the 'and'.",
                             self.default_action : "go",
                                self.canonical_commands : [self.command_from_dict({'Source': '',
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
                                                                        'Name': self.canonical_robot_name, 'Description': []}}}) for w in self.advanced_regions],
                               self.alternate_commands : [self.command_from_dict({'Source': '',
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
                               self.next: "defuse_simple"
                               },    
                "defuse_simple" : {self.prompt : "Junior needs to defuse a bomb. Tell him to do so.",
                                   self.default_action : "go",
                                   self.canonical_commands : [self.command_from_dict({'Source': '',
                                          'Theme': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1},
                                                               'Name': 'bomb',
                                                               'Description': []}},
                                          'Condition': '',
                                          'Patient': '',
                                          'Location': '',
                                          'Action': 'defuse',
                                          'Destination': '',
                                          'Negation': False,
                                          'Agent': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1},
                                                               'Name': self.canonical_robot_name, 'Description': []}}})],
                                   self.alternate_commands : [self.command_from_dict({'Source': '',
                                          'Theme': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1},
                                                               'Name': 'bomb',
                                                               'Description': []}},
                                          'Condition': '',
                                          'Patient': '',
                                          'Location': '',
                                          'Action': 'defuse',
                                          'Destination': '',
                                          'Negation': False,
                                          'Agent': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1},
                                                               'Name': None, 'Description': []}}})],
                                    self.response: "Good work! Junior will defuse the bomb.",
                                    self.next: "defuse_moderate"     
                                    },
                "defuse_moderate" : {self.prompt : "You can tell Junior to do two different things in one sentence. Junior needs to go to the conservatory and defuse a bomb. Tell him to do both in one sentence.",
                                   self.default_action : "go",
                                   self.canonical_commands : [ self.command_from_dict({'Source': '', 
                                            'Theme': '', 
                                            'Condition': '', 
                                            'Patient': '', 
                                            'Location': {'Location': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1}, 
                                                                      'Name': 'conservatory', 'Description': []}}, 
                                            'Action': 'go', 
                                            'Destination': '', 
                                            'Negation': False, 
                                            'Agent': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1}, 
                                                                 'Name': self.canonical_robot_name, 'Description': []}}}),
                                            self.command_from_dict({'Source': '', 
                                            'Theme': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1}, 
                                                                 'Name': 'bomb', 'Description': []}}, 
                                            'Condition': '', 
                                            'Patient': '', 
                                            'Location': '', 
                                            'Action': 'defuse', 
                                            'Destination': '', 
                                            'Negation': False, 
                                            'Agent': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1}, 
                                                                 'Name': self.canonical_robot_name, 'Description': []}}})],
                                    self.alternate_commands : [ self.command_from_dict({'Source': '', 
                                            'Theme': '', 
                                            'Condition': '', 
                                            'Patient': '', 
                                            'Location': {'Location': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1}, 
                                                                      'Name': 'conservatory', 'Description': []}}, 
                                            'Action': 'go', 
                                            'Destination': '', 
                                            'Negation': False, 
                                            'Agent': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1}, 
                                                                 'Name': None, 'Description': []}}}),
                                            self.command_from_dict({'Source': '', 
                                            'Theme': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1}, 
                                                                 'Name': 'bomb', 'Description': []}}, 
                                            'Condition': '', 
                                            'Patient': '', 
                                            'Location': '', 
                                            'Action': 'defuse', 
                                            'Destination': '', 
                                            'Negation': False, 
                                            'Agent': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1}, 
                                                                 'Name': None, 'Description': []}}})],
                                    self.response: "Good work! Junior will go to the conservatory and defuse the bomb.",
                                    self.next: "login"     
                                    },                           
                   "conditional_simple" : {self.prompt : "Tell Junior that if he sees a bomb, defuse it.",
                                           self.canonical_sentence: "If you see a bomb, defuse the bomb.",
                                           self.canonical_commands : [self.command_from_dict({'Source': '',
                                                    'Theme': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1},
                                                                         'Name': 'bomb', 'Description': []}},
                                                    'Condition': {'Command': {'Source': '',
                                                                              'Theme': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': False, 'Number': 1},
                                                                                                   'Name': 'bomb', 'Description': []}},
                                                                              'Condition': '', 
                                                                              'Patient': '', 
                                                                              'Location': '', 
                                                                              'Action': 'see', 
                                                                              'Destination': '', 
                                                                              'Negation': False, 
                                                                              'Agent': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1}, 
                                                                                                   'Name': 'you', 'Description': []}}}}, 
                                                    'Patient': '',
                                                    'Location': '', 
                                                    'Action': 'defuse', 
                                                    'Destination': '', 
                                                    'Negation': False, 
                                                    'Agent': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1}, 
                                                                         'Name': self.canonical_robot_name, 'Description': []}}})],
                                           self.alternate_commands : [self.command_from_dict({'Source': '',
                                                    'Theme': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1},
                                                                         'Name': 'bomb', 'Description': []}},
                                                    'Condition': {'Command': {'Source': '',
                                                                              'Theme': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': False, 'Number': 1},
                                                                                                   'Name': 'bomb', 'Description': []}},
                                                                              'Condition': '', 
                                                                              'Patient': '', 
                                                                              'Location': '', 
                                                                              'Action': 'see', 
                                                                              'Destination': '', 
                                                                              'Negation': False, 
                                                                              'Agent': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1}, 
                                                                                                   'Name': 'you', 'Description': []}}}}, 
                                                    'Patient': '',
                                                    'Location': '', 
                                                    'Action': 'defuse', 
                                                    'Destination': '', 
                                                    'Negation': False, 
                                                    'Agent': {'Object': {'Quantifier': {'Type': 'exact', 'Definite': True, 'Number': 1}, 
                                                                         'Name': None, 'Description': []}}})],
                                    self.response: "Good work! If Junior sees a bomb, he will defuse the bomb.",
                                    self.next: "login"
                                    }                            
                }
        
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
        if d["Theme"] != "":
            theme_object = d["Theme"]["Object"]
            theme = ObjectEntity.from_dict(theme_object)
        else:
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

        action = d["Action"]
        if action in ACTION_ALIASES:
            action = ACTION_ALIASES[action]

        return Command(agent,theme,patient,location,source,destination,action,condition,d["Negation"])
