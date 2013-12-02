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

class Go(object):
    prompt = "prompt"
    necessary_tups = "necessary_tups"      
    response = "response"
    location = "Location"
    destination = "Destination"
    source = "Source"
    locations = [location,destination,source]
    agent_object = "object"
    agents = [agent_object]
    action = "Action"
    default_action = "default_action"
    regions = {   "office" : "office" ,
                    "conservatory" : "conservatory",
                    "cellar" : "cellar"
                }
    ccs = { "and" : "and"}
    dict = {"go_simple" : {prompt : "Tell Junior to go to the %s" % regions["office"],
                           default_action : "go",
                           necessary_tups : [(action,"go",1),
                                              (location,regions["office"],1)
                                              ],
                           response: "Good work! Junior will go."
                           },
            "go_medium" : {prompt : "Tell Junior to go to the office and the conservatory.",
                           default_action : "go",
                           necessary_tups : [(action,"go",1),
                                              (location,regions["office"],1),
                                              (location,regions["conservatory"],1)
                                              ],
                           response: "Good work! Junior will go."
                           },    
            "go_list" : {prompt : "Tell Junior to go to the office, the conservatory, and the cellar.",
                         default_action : "go",
                           necessary_tups : [(action,"go",1),
                                              (location,regions["office"],1),
                                              (location,regions["conservatory"],1),
                                              (location,regions["cellar"],1),
                                              ("cc",ccs["and"],1),
                                              (",",",",2)
                                              ],
                           response: "Good work! Junior will go."
                           }          
            }
    
def check_token_count(slot,count,token,response_list):
    if slot["Name"] == token and response_list.count(token) == count:
        return True
    return False
    

def main():
    go = Go()
    handler = SemanticsHandler()
    for exercise in go.dict:
        print "%s exercise." % exercise
        success = False
        exercise = go.dict[exercise]
        action = exercise[go.default_action]        
        while not success:
            found_tups = []
            success = True
            response = raw_input(exercise[go.prompt]+":\n")
            response_list = response.strip(".").replace(","," ,").lower().split(" ")
            frames = handler.get_semantic_structures(response)
            for frame in frames:                
                command_frame = frame.__dict__()["Command"]
                if not action in response_list:
                    print "Please remember to tell Junior to %s." % action
                    success = False
                for param, token, count in exercise[go.necessary_tups]:
                    if (param, token, count) in found_tups:
                        #already found this tuple
                        pass
                    if param in command_frame:
                        slot = command_frame[param]
                        ##Check action
                        if param == go.action and response_list.count(token) == count:
                            #Correct action and count
                            pass
                        elif param == go.action:
                            #Incorrect action count
                            success = False
                        ##Check Location
                        elif param in go.locations:                        
                            if go.location in slot:
                                slot = slot[go.location]
                            else:
                                #non-location location => failure
                                success = False
                        ##Check Agent
                        elif param in go.agents:
                            if go.agent_object in slot:
                                slot = slot[go.agent_object]
                            else:
                                #non-object agent => failure
                                print "Unacceptable Agent: %s" % slot.items()[0]
                                success = False
                        #Check counts
                        if param != go.action and check_token_count(slot,count,token,response_list):
                            #Correct count
                            pass
                        elif param != go.action:
                            success = False
                            print "Incorrect count of %s as %s, please try again" % (token, param)
                        if success:
                            found_tups.append((param, token, count))
                    else:
                        success = False
            if all([True for tup in exercise[go.necessary_tups] if tup in found_tups]):
                print exercise[go.response]
    
if __name__=="__main__":
    main()
        

        
        