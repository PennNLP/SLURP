"""A handler to process stored semantic frames and sentences
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

from pymongo import MongoClient
import datetime

class Sentence(object):
    """The purpose of this class is to instantiate entries from the status xml file.
        This class should be able to encapsulate anything from semantics.parsing create_semantic_structures
    """
    def __init__(self,params):
        self.sentence_text = ""
        
    @staticmethod
    def to_dictionary_structure(semantic_structure):
        """Semantic representations are equivalent to commands or assertions"""
        return semantic_structure.to_dict()  

class MongoHandler(object):
    user_id_tag = "user_id"
    user_name_tag = "user_name"        
    sentence_tag = "sentence"
    datetime_tag = "datetime"
    exercise_tag = "exercise"
    response_tag = "response"
    user_input_tag = "user_input"
    datetime_tag = "datetime"
    command_tag = "Command"
    default_db_loc = 'mongodb://localhost:27017'
    default_user_id = 1
    default_user_name = "default_user"
    def __init__(self,user_name,user_id,password="default_password"):
        client = MongoClient(self.default_db_loc)        
        self.db = client.slurp_db
        self.collection = self.db.semantics_training_input
        self.user_id = user_id
        self.user_name = user_name
        
    def set_user(self,user_id=None,user_name=None):
        if user_name:
            self.user_name = user_name
        if user_id:
            self.user_id = user_id
            
    def get_all_commands_by_sent(self,sentence):
            return [w[self.command_tag] for w in self.collection.find({self.sentence_tag : sentence})]            
        
    def log_sentence_and_structure(self,sentence,structure):
        d_structure = Sentence.to_dictionary_structure(structure)
        for key in d_structure:
            self.collection.insert({self.user_id_tag : self.user_id, self.user_name_tag : self.user_name, \
                                    self.sentence_tag : sentence, key : d_structure[key], self.datetime_tag: datetime.datetime.utcnow()})
        print d_structure
        
    def log_exercise_and_input(self,uuid,datetime,response,exercise,user_input):
        self.collection.insert({self.user_id_tag : uuid,
                                self.datetime_tag : datetime,
                                self.exercise_tag : exercise,
                                self.response_tag : response,
                                self.user_input_tag : user_input})
        
    def get_all_sents_by_user(self,user_id=None,user_name=None):
        if user_id and user_name:
            return [w["sentence"] for w in self.collection.find({self.user_id_tag : user_id, self.user_name_tag : user_name})]            
        elif user_name:
            return [w["sentence"] for w in self.collection.find({self.user_name_tag : user_name})]        
        elif user_id:
            return [w["sentence"] for w in self.collection.find({self.user_id_tag : user_id})]            
        else:
            return [w["sentence"] for w in self.collection.find({self.user_id_tag : self.default_user_id, self.user_name_tag : self.default_user_name})]
