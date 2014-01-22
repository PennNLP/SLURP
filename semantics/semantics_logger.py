"""An interface for logging and tracking the semantic status of sentences.
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
from semantics.mongo_handler import MongoHandler, Sentence
import datetime

class SemanticsLogger(object):
    ALL_SENTS_COMMAND = "/allsents"
    USER_SENTS_COMMAND = "/usersents"
    COMMANDS_BY_SENT = "/getcommands"
    SET_USER_ID_COMMAND = "/setuserid"
    SET_USER_NAME_COMMAND = "/setusername"
    
    def __init__(self,user_name="default_user",user_id=1):
        self.current_user_name = user_name
        self.current_user_id = user_id
        self.mdb = MongoHandler(user_name,user_id)
        self.sent = Sentence("test_sent")
        self.command_dict = {
                        "/allsents" : { "function" : self.get_all_sents, "param" : []}, 
                        "/usersents" : { "function" : self.get_all_sents, "param" : []},
                        "/getcommands" : { "function" : self.get_all_commands, "param" : ["sentence"]},
                        "/setuserid" : { "function" : self.set_user, "param" : ["user_id"]},
                        "/setusername" : { "function" : self.set_user, "param" : ["user_name"]},                     
                }
        
    def is_command(self,text):
        split = text.split(" ")
        command = split[0]
        args = None
        if len(split) > 1:
            args = split[1:]
        if command in self.command_dict:
            if args:
                return self.command_dict[command]["function"](" ".join(args))
            else:
                return self.command_dict[command]["function"]()
        else:
            return False        
            
    def get_all_sents(self,user_id=None,user_name=None):
        return self.mdb.get_all_sents_by_user(user_id, user_name)
    
    def get_all_commands(self,sentence=None):
        if sentence:
            sentence = " ".join(sentence)
            return self.mdb.get_all_commands_by_sent(sentence)
    
    def set_user(self,user_id=None,user_name=None):
        if user_id:
            self.current_user_id = user_id
            self.mdb.set_user(user_id=user_id)            
        if user_name:
            self.current_user_name = user_name
            self.mdb.set_user(user_name=user_name)
    
    def log_structures(self,sentence,semantic_structures):
        """Log the semantic structures for the sentence"""
        for semantic_structure in semantic_structures:
            self.mdb.log_sentence_and_structure(sentence,semantic_structure)

    def log_semantics_training_input(self,uuid,response,exercise,user_input):
        self.mdb.log_exercise_and_input(uuid,datetime.datetime.now(),response,exercise,user_input)
