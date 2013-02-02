"""Test ltlgeneration.py"""

from ltlgeneration import LTLGenerator

def test():
    """Test LTL Generation."""
    rooms = ["hall", "classroom", "outer_lab"]
    room_adjacency =  (("hall", "classroom"), ("hall", "outer_lab"))
    gen = LTLGenerator(rooms, [("hall")], [('user', 'defuser')], room_adjacency)
    gen.generate("hall", {}, "ltlgen", "..")

if __name__ == "__main__":
    test()