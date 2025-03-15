from jenga_stack import JengaStack, JengaError
from jenga_ui import JengaViewer

import time

jenga = JengaStack(levels=8)

viewer = JengaViewer(jenga.stack)


while True:
    print("level, index, top_index: ")
    level = int(input())
    index = int(input())
    top_index = int(input())
    try:
        jenga.remove_block_and_place_on_top(level=level, remove_index=index, top_placement_index=top_index)
        jenga.print_stack()            
    except JengaError as e:
        print(e)
    
    viewer.update_screen(jenga.stack)

