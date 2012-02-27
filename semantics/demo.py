#!/usr/bin/env python

import Parsing
from Knowledge import Knowledge
from Structures import Entity, Predicate

if __name__ == '__main__':
    ## = Parses correctly
    demo_sentences = {
        'LookForUser' : """((S (VP (VB Look) (PP-CLR (IN for) (NP-A (NNP User)  (CD 1))) (PP (IN in) (NP-A (NP (DT the) (NN library)) (CC and) (NP (DT the) (NN classroom))))) (. .)))""",
        'CallIfFlipped' : """(S (CC And) (SBAR-ADV (IN if) (S-A  (NP-SBJ-A-0 (PRP you)) (VP (VBP get) (VP-A (VBN flipped) (NP-A-0 (-NONE- *))))))(, ,) (NP-SBJ-A (-NONE- *)) (VP (VB call) (NP-A (PRP me)))(. .))""",
        'GoToLibrary' : """((S (NP-SBJ-A (-NONE- *)) (VP (VB Go) (PP-CLR (TO to) (NP-A (DT the) (NN library)))) (. .))) """,
        'ThereAreTwoHostages' : """(S (NP-SBJ-A (EX There)) (VP (VBP are) (NP-PRD-A (NP (CD two) (NNS hostages)) (PP-LOC (IN in) (NP-A (DT the) (NN library))))) (. .))""",
        'WhereAreTheHostages' : """((SBARQ (WHADVP-0 (WRB Where)) (SQ (VP (VBP are) (NP-PRD-A (DT the) (NNS hostages)) (ADVP-0 (-NONE- *T*)))) (. ?)))""",
        #'TellSeeBombHostage' : """(S  (NP-SBJ-A (-NONE- *)) (VP (VB Tell) (NP-A (PRP me)) (SBAR-ADV (IN if) (S-A  (NP-SBJ-A (PRP you)) (VP (VBP see) (NP-A  (NP (DT a) (NN bomb))(CC or) (NP (DT a) (NN hostage)))))))(. .))""",
        'GoToRoom3' : """((S (NP-SBJ-A (-NONE- *)) (VP (VB Go) (PP-CLR (TO to) (NP-A (NN room) (CD 3)))) (. .))) """,
        'DriveToTheHallway' : """((S (NP-SBJ-A (-NONE- *)) (VP (VB Drive) (PP-CLR (TO to) (NP-A (DT the) (NN hallway)))) (. .))) """,
        '' : """This is just so you can comment out test sentences without worrying about a comma."""
        }
        
    #parse_tree_string = """(S  (NP-SBJ-A (-NONE- *)) (VP (VB Defuse) (NP-A  (NP (DT any) (NNS bombs)) (SBAR (IN that) (S-A  (NP-SBJ-A (PRP you)) (VP  (VP-A (VBP see))(CC and) (VP-A (VBP notify) (NP-A (PRP me))))))))(. .))"""
    # ^ Bad parse
    ##parse_tree_string = """(S  (NP-SBJ-A (-NONE- *)) (VP (VB Tell) (NP-A (PRP me)) (SBAR-ADV (IN if) (S-A  (NP-SBJ-A (PRP you)) (VP (VBP see) (NP-A  (NP (DT a) (JJ bad) (NN guy))(CC or) (NP (DT a) (NN hostage)))))))(. .))"""
    ##parse_tree_string = """(S (NP-SBJ-A (-NONE- *)) (VP (VB Tell) (NP-A (PRP me))) (. .))"""
    #parse_tree_string = """((S (NP-SBJ-A (-NONE- *)) (VP(VB Tell) (NP-A (PRP me)) (SBAR-A(WHNP-0 (WDT what)) (S-A(NP-SBJ-0 (-NONE- *T*)) (VP (VBZ 's) (PP-LOC-PRD (IN in) (NP-A (NN room) (CD 3)))))))                                                 (. ?)))"""

    ##parse_tree_string = """((S (NP-SBJ-A (PRP I)) (VP (VBP am) (PP-LOC-PRD (IN in) (NP-A (NN room) (CD 2)))) (. .)))"""
    ##parse_tree_string = """((S (NP-SBJ-A (EX There)) (VP (VBP are) (NP-PRD-A (NP (QP (IN at) (JJS least) (CD 4)) (NNS hostages)) (PP-LOC (IN in) (NP-A (NN room) (CD 1))))) (. .)))  """
  
    
    ##parse_tree_string = """((SBARQ (WHNP-0 (WP What)) (SQ (VBP are) (NP-SBJ (PRP you)) (VP (VBG doing) (NP-0 (-NONE- *T*)))) (. ?)))"""
    #parse_tree_string = """((SQ (VBP Are) (NP-SBJ-0 (PRP you)) (VP (VBN flipped) (NP-A-0 (-NONE- *))) (. ?)))"""
    ##parse_tree_string = """((SINV (VP (VBP Are) (ADVP-LOC-PRD (RB there))) (NP-SBJ (DT any) (NNS hostages)) (. ?))) """

    ##parse_tree_string = """((S (NP-SBJ-A (NNP Junior)) (, ,) (VP (VP-A (VB go) (PP-CLR (TO to) (NP-A (NN room) (CD 3)))) (. .))))"""
    ##parse_tree_string = """((S (VP (VP-A (VB Go) (PP-CLR (TO to) (NP-A (NN room) (CD 3)))) (. .))))"""
    ##parse_tree_string = """((S (NP-SBJ-A (-NONE- *)) (VP (VB Look) (PP-CLR (IN for) (NP-A (NNP User) (CD 1))) (PP (IN in) (NP-A (NP (DT the) (NN library)) (CC and) (NP (DT the) (NN classroom))))) (. .)))"""
    ##parse_tree_string = """((S (VP (VB Look) (PP-CLR (IN for) (NP-A (NNP User) (CD 1))) (PP (IN in) (NP-A (NP (DT the) (NN library)) (CC and) (NP (DT the) (NN classroom))))) (. .)))"""
                                        
    knowledge = Knowledge()
    
    print knowledge.process_parse_tree(demo_sentences['GoToLibrary'], '')
    #print knowledge.process_parse_tree(demo_sentences['WhereAreTheHostages'], '')
    
    print knowledge.watch_list
    print knowledge.commander_known_entities
    print knowledge.command_queue
    

    #print 'Watch list \n' + str(knowledge.watch_list)
    print '-------------'

    
    
    ##parse_tree_string = """((SBARQ (WHADVP-0 (WRB Where)) (SQ (VP (VBP are) (NP-PRD-A (DT the) (NNS hostages)) (ADVP-0 (-NONE- *T*)))) (. ?)))  """
    #parse_tree_string = """((S (NP-SBJ-A (-NONE- *)) (VP (VB Go) (PP-CLR (TO to) (NP-A (NP (DT the) (NN room)) (PP (IN with) (NP-A (DT the) (NNS hostages)))))) (. .))) """
    ##parse_tree_string = """((SINV (VP (VBP Are) (ADVP-LOC-PRD (RB there))) (NP-SBJ (NP (DT any) (NNS hostages)) (PP-LOC (IN in) (NP-A (NN room) (CD 3)))) (. ?)))"""
    #result = Parsing.get_semantics_from_parse_tree(parse_tree_string)

    #print result
    
    #semantic_structures = (Parsing.create_semantic_structures(result))

    #print semantic_structures
    
    #answer = knowledge.parse_semantic_structures(semantic_structures)
    #print answer
    

    
