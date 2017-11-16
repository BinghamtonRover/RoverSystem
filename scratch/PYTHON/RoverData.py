import json
import logging
import logging.handlers
import time

class RoverData:
    def __init__(self, name):
        self.name = name
        self.temperature = 46.6
        self.latitude = 5046.31972
        self.latitudeDirection = "N"
        self.longitude = 16.3365
        self.longitudeDirection = "E"
        self.pressure = 3.6
        
    def toJSON(self):
        #Returns a string in JSON format
        #When passed into the Logger it prints to file
        return json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=4)

    def createLogger(self, fileName):
        #Creating and setting up the logger
        logger = logging.getLogger("Timed Logger")
        logger.setLevel(logging.INFO)
        handler = logging.handlers.TimedRotatingFileHandler(fileName, when="s", interval = 1, backupCount=5)
        logger.addHandler(handler)
              
        #This is me adding to the logger
        #I update an element (name) to check if it
        #correctly creates updated files
        for i in range(50000):
            if i % 50 == 0:
                self.temperature += 98
            else:
                self.temperature -= 2
            logger.info(self.toJSON())
