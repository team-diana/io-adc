#!/usr/bin/python

import io_card_python

card = io_card_python.io_card_python()

card.open()

stop = False
print ("enter 'q' or 'exit' to finish the program")

last={ 
    8: False,
    9: False,
    10: False,
    11: False,
    12: False,
    13: False,
    14: False,
    15: False
}

while not stop:
    s = raw_input("-->:")
    if s == "q" or s == "exit":
        stop = True
    else:
        s = int(s)
        if 8 <= s <= 15:
            lastValue = last[s]
            newValue = not lastValue
            last[s] = newValue
            print ("setting to " + str(newValue))
            card.write_line(s, newValue) 

card.close()
