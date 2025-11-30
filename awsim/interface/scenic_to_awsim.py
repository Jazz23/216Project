import json
from pathlib import Path

class ScenicToAwsim:
    def __init__(self):
        print("Interface Initialized")
        # @Graeme, I left these (below) as type hints for future reference
        self.scenic_path: str | None = None
        self.converted_data: dict | None = None
    
    #Loading a scenic scenario file
    def load_scenic(self, scenic_path: str):
        self.scenic_path = scenic_path
        print(f"Loading Scenic Scenario from: {scenic_path}")
        # -> actually read/parse the file, tokens/regex? haven't thought it through yet
        
    #converting scenic scenario into an AWSIM-compatible format
    def convert(self):
        if self.scenic_path is None:
            raise ValueError("File not loaded. Call load_scenic() first")
        
        print("Converting Scenic Scenario into AWSIM-compatible format...")
        # Note: create real data structures / JSON
        self.converted_data = {
            "ego": {
                "x": 10,
                "y": 5,
                "heading_deg": 90,
                "speed": 3,
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