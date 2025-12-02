''' This file is a bsic test for the AWSIM Simulator

    This file confirms that the "AwsimSimulator" class can be imported, constructed, and run without errors.
    It does NOT test ROS2 or any true AWSIM integration (@Graeme: that's where you come in) only that the 
    Python interface layer is valid and that the placeholder logic executes.
    
    Once you [Graeme] implement the actual AWSIM backend, this test can be reworked to check:
    - object addition
    - stepping via the simulator
    - retrieval of object states
    - correct shutdown behavior
'''


from awsim.interface.awsim_simulator import AwsimSimulator

def main():
    dummy_config = {}
    sim = AwsimSimulator(dummy_config)
    print("AwsimSimulator initialized (placeholder).")
    
if __name__ == "__main__":
    main()