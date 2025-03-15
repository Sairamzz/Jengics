class JengaError(Exception):
    """Base class for all Jenga-related errors."""
    pass

class BlockAlreadyRemovedError(JengaError):
    """Raised when attempting to remove a block that has already been removed."""
    def __init__(self, level, index):
        super().__init__(f"Block at Level {level}, Position {index} is already removed!")

class BlockAlreadyPresentError(JengaError):
    """Raised when attempting to place a block that already exists."""
    def __init__(self, level, index):
        super().__init__(f"Block at Level {level}, Position {index} is already present!")

class InvalidJengaMoveError(JengaError):
    """Raised when an invalid level or index is used."""
    def __init__(self, level, index):
        super().__init__(f"Invalid move: Level {level}, Position {index} is out of bounds.")

class NoBlockRemovedError(JengaError):
    """Raised when no block is removed but is tried to place on top of stack"""
    def __init__(self):
        super().__init__(f"No blocks available to place, Remove Block First")   

class InvalidIndexError(JengaError):
    """Raised when an invalid index is passed which is less than 0 or greater than 2"""
    def __init__(self):
        super().__init__(f"Invalid index, Use values 0, 1, or 2")   


class JengaStack:
    def __init__(self, levels=6):
        self.levels = levels
        self.stack = self._initialize_stack()
        self.moved_blocks = []

    def _initialize_stack(self):
        return [[True, True, True] for _ in range(self.levels)]

    def _can_add_new_level(self):
        return all(self.stack[-1])  # Returns True if top level has no False values

    def _append_level(self):
        new_level = [False, False, False]
        self.stack.append(new_level)
        self.levels += 1

    def _check_illegal_move(self, level, index):
        """Checks if removing the block would leave an empty level, excluding the top level"""
        if level == self.levels:
            return True  

        if 1 <= level <= self.levels and 0 <= index < 3:
            remaining_blocks = sum(self.stack[level - 1])

            if remaining_blocks == 1 and self.stack[level - 1][index] is True:
                print(f"Removing block at Level {level}, Position {index} would leave the level empty!")
                return False
        return True


    def remove_block_and_place_on_top(self, level, remove_index, top_placement_index):

        if not (1 <= level <= self.levels and 0 <= remove_index < 3):
            # print("Invalid level or index. Please check the input.")
            raise InvalidJengaMoveError(level, remove_index)
        
        if not self._check_illegal_move(level, remove_index):
            return False
        
        if self.stack[level - 1][remove_index] is False:
            # print(f"Block at Level {level}, Position {remove_index} is already removed!")
            raise BlockAlreadyRemovedError(level, remove_index)

        self.stack[level - 1][remove_index] = False
        self.moved_blocks.append(remove_index)
        
        try:
            self._place_block_on_top(top_placement_index)
        except JengaError as e:
            self.stack[level - 1][remove_index] = True
            self.moved_blocks.pop(0)            
            raise BlockAlreadyPresentError(level, top_placement_index)


    def _place_block_on_top(self, index):

        if not self.moved_blocks:
            # print("No blocks available to place. Remove a block first.")
            raise NoBlockRemovedError()


        if all(self.stack[-1]):
            self._append_level()

        top_level = self.levels

        if 0 <= index < 3:

            if self.stack[top_level - 1][index] is False:
                self.stack[top_level - 1][index] = True
                self.moved_blocks.pop(0)
            else:
                # print(f"Position {index} on Level {top_level} is already occupied.")
                raise BlockAlreadyPresentError(top_level, index)
        else:
            # print("Invalid index. Use values 0, 1, or 2.")
            raise InvalidIndexError()
            
    def print_stack(self):
        print()
        # print(self.stack)
        for level in range(self.levels - 1, -1, -1):  # Print from top to bottom
            print(f"Level {level + 1}: {self.stack[level]}")

# # Example usage:
# if __name__ == "__main__":
#     jenga = JengaStack(levels=6)

#     while True:
        
#         print("level, index, top_index: ")
#         level = int(input())
#         index = int(input())
#         top_index = int(input())
#         try:
#             jenga.remove_block_and_place_on_top(level=level, remove_index=index, top_placement_index=top_index)
#             jenga.print_stack()            
#         except JengaError as e:
#             print(e)
        