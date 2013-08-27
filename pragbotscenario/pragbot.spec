# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pick_up, 0
drop, 0
radio, 0
extinguish, 0

CompileOptions:
convexify: False
parser: slurp
fastslow: False
decompose: False
use_region_bit_encoding: True

CurrentConfigName:
pragbot

Customs: # List of custom propositions
carrying_item

RegionFile: # Relative path of region description file
pragbot.converted.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
fire, 0
person, 0
hazardous_item, 0


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
classroom = classroom
conservatory = conservatory
bathroom = bathroom
cellar = cellar
ballroom = ballroom
office = office
annex = annex
hallway2 = hallway2
hallway3 = hallway3
study = study
hallway1 = hallway1
computer_room = computer_room
lounge = lounge
billiard_room = billiard_room
hallway4 = hallway4
dining_room = dining_room
library = library
kitchen = kitchen

Spec: # Specification in structured English


