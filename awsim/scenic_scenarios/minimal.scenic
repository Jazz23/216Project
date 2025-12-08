param egoSpeed = 2.0    # initial speed parameter

ego = new Object at (0, 0), facing 90 deg, with name "ego"

npc1 = new Object behind ego by 10, facing ego.heading, with name "npc1"
npc2 = new Object behind npc1 by 10, facing npc1.heading, with name "npc2"