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

class SemanticsLogger(object):
    def __init__(self,user_name="default_user",user_id=1):
        self.current_user_name = user_name
        self.current_user_id = user_id
        self.mdb = MongoHandler(user_name,user_id)
        self.sent = Sentence("test_sent")
        
    def get_all_sents(self,user_id=None,user_name=None):
        return self.mdb.get_all_sents(user_id, user_name)
    
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
        
        
        