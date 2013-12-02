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

from debug_semantics import SemanticsHandler
from semantics.new_structures import Command, ObjectEntity, Location

import unittest 

class Go(unittest.TestCase):
    def setUp(self):
        self.handler = SemanticsHandler()
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
                               self.response: "Good work! Junior will go."
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
                               self.response: "Good work! Junior will go."
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
                               self.response: "Good work! Junior will go."
                               }          
                }

#     def test_go_moderate(self):
#         exercise = self.train_dict["go_moderate"]
#         success = False
#         while not success:
#             success = True
#             response = self.get_input(exercise[self.prompt]+":\n")
#             commands = self.handler.get_semantic_structures(response)            
#             for command in commands:                
#                 self.assertIn(str(command),[str(w) for w in exercise[self.correct_commands]])
                        
    def test_go_simple(self):
        exercise = self.train_dict["go_simple"]
        success = False
        while not success:
            success = True
            response = self.get_input(exercise[self.prompt]+":\n")
            commands = self.handler.get_semantic_structures(response)            
            for command in commands:                
                self.assertIn(command,exercise[self.correct_commands])
                
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

if __name__=="__main__":
    unittest.main()
        

        
        