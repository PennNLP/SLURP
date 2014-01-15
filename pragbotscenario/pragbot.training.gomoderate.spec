# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
defuse, 1

CompileOptions:
convexify: False
parser: slurp
fastslow: False
decompose: False
use_region_bit_encoding: True
slurp_restrict_actions: True

CurrentConfigName:
pragbotTrainingModerate

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
pragbot.training.gomoderate.converted.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bomb, 1
hostage, 1
badguy, 1
defuse_done, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
conservatory = conservatory
kitchen = kitchen
office = office
lab = lab
west_hallway = west_hallway
east_hallway = east_hallway
entrance = entrance

Spec: # Specification in structured English


