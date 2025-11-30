from awsim.interface.scenic_to_awsim import ScenicToAwsim

def main():
    inter = ScenicToAwsim()
    inter.load_scenic("scenic_scenarios/lane_test.scenic")
    inter.convert()
    inter.export_config("config_templates/lane_test_config.json")
    
if __name__ == "__main__":
    main()