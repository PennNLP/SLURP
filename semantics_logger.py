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
        return semantic_structure.__dict__()
        
class MongoHandler(object):
    def __init__(self,user_name,user_id):
        client = MongoClient('mongodb://localhost:27017')
        self.db = client.slurp_db
        self.collection = self.db.sentence_semantics_collection
        self.user_id = user_id
        self.user_name = user_name
        
    def log_sentence_and_structure(self,sentence,structure):
        d_structure = Sentence.to_dictionary_structure(structure)
        for key in d_structure:
            self.collection.insert({"user_id" : self.user_id, "user_name" : self.user_name, \
                                    "sentence" : sentence, key : d_structure[key], "datetime": datetime.datetime.utcnow()})
        print d_structure
        
        
class SemanticsLogger(object):
    def __init__(self,user_name="default_user",user_id=1):
        self.mdb = MongoHandler(user_name,user_id)
        self.sent = Sentence("test_sent")
    
    def log_structures(self,sentence,semantic_structures):
        """Log the semantic structures for the sentence"""
        for semantic_structure in semantic_structures:
            self.mdb.log_sentence_and_structure(sentence,semantic_structure)
        
        
        