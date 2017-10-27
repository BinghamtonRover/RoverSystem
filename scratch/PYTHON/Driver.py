import RoverData

def main():
    rover = RoverData.RoverData("Mars Rover 3.15")
    rover.createLogger('rotating_file.log')

main()
