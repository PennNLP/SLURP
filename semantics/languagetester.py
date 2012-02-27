#!/usr/bin/env python

import Parsing
from Knowledge import Knowledge
from Structures import Entity, Predicate

"""To use this tester:
1. Add the parse tree to the phrase dict .
2. Add a tuple to the test case as follows:

    (<phrase_dict key>, <desired test information>, <desired result string>)

    where <desired test information> is:
    0: Command Queue
    1: Watch List (for events)
    2: Commander known entities

3. Think about how much time Ian Perera saved by using string comparisons before you yell at him
    for using string comparisons.
"""


phrase_dict = {}

phrase_dict['WhereAreYou'] = """
(SBARQ
    (WHADVP-0 (WRB Where)) 
    (SQ
        (VBP are)
        (NP-SBJ (PRP you))
        (ADVP-0 (-NONE- *T*)))(. ?))"""

phrase_dict['EatSandwich'] = """(
(S
(NP-SBJ-A (NNP Junior))
(, ,)
(VP (VBP eat) (NP-A (DT a) (NN sandwich)))
(. .)))"""

phrase_dict['TellSeeBombHostage'] = """(                                            
(S                                         
(NP-SBJ-A (-NONE- *))                    
(VP                                      
  (VB Tell)                              
  (NP-A (PRP me))                        
  (SBAR-ADV                              
    (IN if)                              
    (S-A                                 
      (NP-SBJ-A (PRP you))               
      (VP                                
        (VBP see)                        
        (NP-A                            
          (NP (DT any) (NNS bombs))      
          (CC or)                        
          (NP (NNS hostages)))))))       
(. .)))"""

phrase_dict['FollowMe'] = """( (S (NP-SBJ-A (-NONE- *))
                                    (VP (VB Follow) (NP-A (PRP me))) (. .))) """

phrase_dict['LookForUser1'] = """(                                               
(S                                            
(NP-SBJ-A (-NONE- *))                       
(VP                                         
  (VB Look)                                 
  (PP-CLR (IN for) (NP-A (NNP User) (CD 1)))
  (PP                                       
    (IN in)                                 
    (NP-A                                   
      (NP (DT the) (NN library))            
      (CC and)                              
      (NP (DT the) (NN classroom)))))       
(. .)))"""

phrase_dict['GetBombDefuser'] = """(                                                                 
(S                                                              
(NP-SBJ-A (-NONE- *))                                         
(VP                                                           
  (VP-A                                                       
    (VB Get)                                                  
    (NP-A (DT the) (NN bomb) (NN defuser))                    
    (PP-CLR (IN from) (NP-A (NNP User) (CD 1))))              
  (CC and)                                                    
  (VP-A (VB come) (ADVP-DIR (RB back)) (ADVP-TMP (RB here)))) 
(. .))) """

phrase_dict['HallwayDoor'] = """
(                                                           
  (S                                                        
    (NP-SBJ-A (NNP Junior))                                 
    (, ,)                                                   
    (VP                                                     
      (VP-A                                                 
        (VB go)                                             
        (ADVP-DIR (RB down) (NP (DT the) (NN hallway)))     
        (PP-DIR                                             
          (TO to)                                           
          (NP-A                                             
            (NP (DT the) (NN door))                         
            (PP-LOC (IN on) (NP-A (DT the) (NN left))))))   
      (CC and)                                              
      (VP-A                                                 
        (VB defuse)                                         
        (NP-A                                               
          (NP (DT the) (NN bomb))                           
          (PP-LOC (IN in) (NP-A (RB there))))))             
    (. .))) """

phrase_dict['GoOfficeDefuse'] = """
(                                                                 
  (S                                                              
    (NP-SBJ-A (-NONE- *))                                         
    (VP                                                           
      (VP-A (VB Go) (PP-CLR (TO to) (NP-A (DT the) (NN office)))) 
      (CC and)                                                    
      (VP-A                                                       
        (VB defuse)                                               
        (NP-A                                                     
          (NP (DT any) (NNS bombs))                               
          (SBAR                                                   
            (WHNP-0 (-NONE- 0))                                   
            (S-A                                                  
              (NP-SBJ-A (PRP you))                                
              (VP (VBP see) (NP-0 (-NONE- *T*))))))))             
    (. .))) """

phrase_dict['WhereIsUser1'] = """
(                                                                        
  (SBARQ                                                                 
    (WHADVP-0 (WRB Where))                                               
    (SQ (VP (VBZ is) (NP-PRD-A (NNP User1)) (ADVP-0 (-NONE- *T*))))      
    (. ?))) """

phrase_dict['WhereIsHe'] = """
(                                                             
  (SBARQ                                                      
    (WHADVP-0 (WRB Where))                                    
    (SQ (VBZ is) (NP-SBJ (PRP he)) (ADVP-0 (-NONE- *T*)))     
    (. ?)))"""

test_cases = []

test_cases.append(('TellSeeBombHostage', 1, """[Event:
	EntityClass: EntityClass: 
		Quantifier: 
	Plural: None
	Definite: None
	Exhaustive: None
	Proportionality: exact
	Number: None

		Predicates: [[Theme:bomb]]
	Sensor: characterize
	Command: Command: 
	EntityClass: EntityClass: 
		Quantifier: 
	Plural: None
	Definite: True
	Exhaustive: None
	Proportionality: exact
	Number: None

		Predicates: [[Topic:Commander], [Agent:*]]
	Action: tell, Event:
	EntityClass: EntityClass: 
		Quantifier: 
	Plural: None
	Definite: None
	Exhaustive: None
	Proportionality: exact
	Number: None

		Predicates: [[Theme:hostage]]
	Sensor: characterize
	Command: Command: 
	EntityClass: EntityClass: 
		Quantifier: 
	Plural: None
	Definite: True
	Exhaustive: None
	Proportionality: exact
	Number: None

		Predicates: [[Topic:Commander], [Agent:*]]
	Action: tell]"""))

test_cases.append(('FollowMe', 0, """[('follow', {'Theme': 'Commander'})]"""))
test_cases.append(('LookForUser1', 0, """[('search', {'Theme': 'user_1', 'Location': 'library'}), ('search', {'Theme': 'user_1', 'Location': 'classroom'})]"""))
test_cases.append(('GetBombDefuser', 0, """[('retrieve', {'Source': 'user_1', 'Theme': 'bomb_defuser'}), ('go', {'Theme': '*', 'Location': 'unknown_current_location'})]"""))
test_cases.append(('EatSandwich', 0, """[('eat', {'Patient': 'sandwich'})]"""))
test_cases.append(('WhereAreYou', 3, """I don't know."""))
test_cases.append(('GoOfficeDefuse', 0, """[('go', {'Location': 'office'}), ('defuse', {'Theme': 'bomb'}), ('see', {'Theme': 'bomb'})]"""))
test_cases.append(('WhereIsUser1',3,""" """));
test_cases.append(('WhereIsHe',3, """ """));
if __name__ == '__main__':
    knowledge = Knowledge()
    library_entity = Entity({'EntityType':[Predicate('EntityType','room')]},
                             'library')
    user1_entity = Entity({'Theme' : [Predicate('Theme', 'user1')],
                            'Location' :
                               [Predicate('Location', library_entity)]})
    
    knowledge.junior_known_entities[user1_entity.id] = user1_entity
    knowledge.commander_known_entities[user1_entity.id] = user1_entity

    for test in test_cases:
        answer = knowledge.process_parse_tree(phrase_dict[test[0]],'')
        if answer[2] is not None:
            result = str(answer[2][test[1]])
        else:
            print 'Test ' + test[0] + ' failed. No output.'
            continue
        if not (test[2] == result):
            print 'Test ' + test[0] + ' failed.'
            print 'Expected: ' + test[2]
            print 'Got: ' + result
        knowledge.process_parse_tree('','execute')
