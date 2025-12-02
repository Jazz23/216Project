import json
from pathlib import Path
import re

class ScenicToAwsim:
    def __init__(self):
        print("Interface Initialized")
        # @Graeme, I left these (below) as type hints for future reference
        self.scenic_path: str | None = None
        self.converted_data: dict | None = None
        self.scenic_text: str | None = None
    
    #Loading a scenic scenario file
    def load_scenic(self, scenic_path: str):
        self.scenic_path = scenic_path
        path = Path(scenic_path)
        print(f"Loading Scenic Scenario from: {scenic_path}")
        # -> actually read/parse the file, tokens/regex? haven't thought it through yet
        
        if not path.exists():
            raise FileNotFoundError(f"Scenic file not found: {path}")
        
        self.scenic_text = path.read_text(encoding="utf-8")

    # Using regex to grab the x,y, and degree values from the Scenic File
    def _parse_ego_pos(self) -> dict:
        if self.scenic_text is None:
            raise ValueError("No Scenic text loaded. Call load_scenic() first")
        pattern = r"ego\s+at\s*\(\s*(?P<x>[-\d\.]+)\s*,\s*(?P<y>[-\d\.]+)\s*\)\s*,\s*facing\s*(?P<degree>[-\d\.]+)\s*deg"
        match = re.search(pattern, self.scenic_text)
        
        if not match:
            raise ValueError(
                "Could not find ego position in Scenic file."
                "Expected something like: ***insert pattern here****"
            )
        x = float(match.group("x")) # x value
        y = float(match.group("y")) # y value
        d = float(match.group("d")) # degrees
        
        return {"x": x, "y": y, "degree": d}
    
    #converting scenic scenario into an AWSIM-compatible format
    def convert(self):
        if self.scenic_path is None:
            raise ValueError("File not loaded. Call load_scenic() first")
        
        print("Converting Scenic Scenario into AWSIM-compatible format...")
        # Note: create real data structures / JSON
        
        ego_pose = self._parse_ego_pos()
        self.converted_data = {
            "objects": {
                "id": "ego",
                "type": "vehicle",
                "x": ego_pose["x"],
                "y": ego_pose["y"],
                "heading_deg": ego_pose["degree"],
                "speed": 3, #placeholder/parse from scenic later?
            }
        }
    
    #Export converted data to a JSON or config template 
    def export_config(self, output_path: str):
        if self.scenic_path is None:
            raise ValueError("No data converted. Call convert() first")
        
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        print(f"Writing AWSIM config to: {output_path}")
        print("Data:", self.converted_data)
        # -> actually write JSON / config file
        
        with output_path.open("w", encoding="utf-8") as f:
            json.dump(self.converted_data, f, indent=2)