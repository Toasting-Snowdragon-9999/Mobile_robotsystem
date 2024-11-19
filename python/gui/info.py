

class Info:
    def __init__(self) -> None: 
        self.info_levels = {
            -1: " ",
            0: "[ Help ]",
            1: "[ Info ]",
            2: "[ Warning ]",
            3: "[ Error ]",
            4: "[ Critical ]"
        } 

        self.color_codes = {
            -1: " ",
            0: "#dede6d",
            1: "#4c89c7",
            2: "#f9a825",
            3: "#b54343",
            4: "#a32e2e"
        }

    def get_surfix(self, level: int) -> str:
        return self.info_levels.get(level, "[ Unknown ]")
    
    def get_color_code(self, level: int) -> str:
        return self.color_codes.get(level, "#000000") 